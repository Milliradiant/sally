//! An egui context drawn into a texture of its own, on whatever GL context is current: the SteamVR overlay and its
//! dashboard button in the window host (the C++ panel's OverlayView), and the headless renderer's views (tests,
//! screenshots). Each view has its own egui context and its own painter (an egui context's textures live in its
//! painter), all sharing the one GL context.

use std::sync::Arc;

use egui::{Pos2, Rect, Vec2, ViewportId};
use glow::HasContext;

pub struct Offscreen {
	pub ctx: egui::Context,
	painter: egui_glow::Painter,
	gl: Arc<glow::Context>,
	fbo: glow::Framebuffer,
	tex: glow::Texture,
	/// the texture's size (pixels)
	pub width: u32,
	pub height: u32,
	/// pixels a point
	pub ppp: f32,
	/// drawn upside down: the dashboard button's picture (SteamVR shows a button's texture with its first row at the
	/// top, where GL has the bottom, and takes no texture bounds for it)
	pub mirror: bool,
	/// the last frame's times (ms): the panel's UI, egui's tessellation, the painting
	pub last_ms: [f64; 3],
	destroyed: bool,
}

impl Offscreen {
	/// A view `width` x `height` pixels at `ppp` pixels a point, on the current GL context
	pub fn new(gl: Arc<glow::Context>, width: u32, height: u32, ppp: f32) -> Result<Offscreen, String> {
		let painter = egui_glow::Painter::new(gl.clone(), "", None, false).map_err(|e| e.to_string())?;
		// SAFETY: plain GL object creation on the current context; the names are checked and kept for destroy
		let (fbo, tex) = unsafe {
			let tex = gl.create_texture()?;
			gl.bind_texture(glow::TEXTURE_2D, Some(tex));
			gl.tex_image_2d(
				glow::TEXTURE_2D,
				0,
				glow::RGBA8 as i32,
				width as i32,
				height as i32,
				0,
				glow::RGBA,
				glow::UNSIGNED_BYTE,
				glow::PixelUnpackData::Slice(None),
			);
			gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_MIN_FILTER, glow::LINEAR as i32);
			gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_MAG_FILTER, glow::LINEAR as i32);
			gl.bind_texture(glow::TEXTURE_2D, None);
			let fbo = gl.create_framebuffer()?;
			gl.bind_framebuffer(glow::FRAMEBUFFER, Some(fbo));
			gl.framebuffer_texture_2d(glow::FRAMEBUFFER, glow::COLOR_ATTACHMENT0, glow::TEXTURE_2D, Some(tex), 0);
			let status = gl.check_framebuffer_status(glow::FRAMEBUFFER);
			gl.bind_framebuffer(glow::FRAMEBUFFER, None);
			if status != glow::FRAMEBUFFER_COMPLETE {
				gl.delete_framebuffer(fbo);
				gl.delete_texture(tex);
				return Err(format!("framebuffer incomplete ({status:#x})"));
			}
			(fbo, tex)
		};
		Ok(Offscreen {
			ctx: egui::Context::default(),
			painter,
			gl,
			fbo,
			tex,
			width,
			height,
			ppp,
			mirror: false,
			last_ms: [0.0; 3],
			destroyed: false,
		})
	}

	/// The view's size in points
	pub fn size_points(&self) -> Vec2 {
		Vec2::new(self.width as f32 / self.ppp, self.height as f32 / self.ppp)
	}

	/// One frame: `events` in (the view's own pointer and keys), the UI run on the root Ui, painted into the texture.
	/// `time` is the host's clock (s).
	pub fn frame(
		&mut self, time: f64, events: Vec<egui::Event>, ui: impl FnMut(&mut egui::Ui),
	) -> egui::PlatformOutput {
		let size = self.size_points();
		let mut raw = egui::RawInput {
			screen_rect: Some(Rect::from_min_size(Pos2::ZERO, size)),
			time: Some(time),
			events,
			focused: true,
			max_texture_side: Some(self.painter.max_texture_side()),
			..Default::default()
		};
		raw.viewports.entry(ViewportId::ROOT).or_default().native_pixels_per_point = Some(self.ppp);
		let t0 = std::time::Instant::now();
		let mut out = self.ctx.run_ui(raw, ui);
		let t1 = std::time::Instant::now();
		let mut prims = self.ctx.tessellate(std::mem::take(&mut out.shapes), out.pixels_per_point);
		let t2 = std::time::Instant::now();
		if self.mirror {
			mirror_y(&mut prims, size.y);
		}
		let size_px = [self.width, self.height];
		// SAFETY: the framebuffer is this view's, made on this GL context; the default one is bound again after
		unsafe { self.gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo)) };
		self.painter.clear(size_px, [0.0, 0.0, 0.0, 0.0]);
		self.painter.paint_and_update_textures(size_px, out.pixels_per_point, &prims, &mut out.textures_delta);
		// SAFETY: as above; the flush hands the drawing to the driver before another process (SteamVR) reads it
		unsafe {
			self.gl.bind_framebuffer(glow::FRAMEBUFFER, None);
			self.gl.flush();
		}
		let ms = |a: std::time::Instant, b: std::time::Instant| (b - a).as_secs_f64() * 1e3;
		self.last_ms = [ms(t0, t1), ms(t1, t2), ms(t2, std::time::Instant::now())];
		out.platform_output
	}

	/// The texture's GL name, what OpenVR takes as an OpenGL texture
	pub fn gl_name(&self) -> u32 {
		self.tex.0.get()
	}

	/// The picture, top row first
	pub fn read(&self) -> egui::ColorImage {
		// SAFETY: binds this view's framebuffer to read it, then the default one again
		unsafe { self.gl.bind_framebuffer(glow::FRAMEBUFFER, Some(self.fbo)) };
		let image = self.painter.read_screen_rgba([self.width, self.height]);
		// SAFETY: as above
		unsafe { self.gl.bind_framebuffer(glow::FRAMEBUFFER, None) };
		image
	}

	/// Frees the GL objects (while the GL context is still current)
	pub fn destroy(&mut self) {
		if self.destroyed {
			return;
		}
		self.destroyed = true;
		self.painter.destroy();
		// SAFETY: the objects are this view's and are not used after this
		unsafe {
			self.gl.delete_framebuffer(self.fbo);
			self.gl.delete_texture(self.tex);
		}
	}
}

/// The tessellated picture turned upside down within `height` points
fn mirror_y(prims: &mut [egui::ClippedPrimitive], height: f32) {
	for p in prims {
		let r = p.clip_rect;
		p.clip_rect = Rect::from_min_max(Pos2::new(r.min.x, height - r.max.y), Pos2::new(r.max.x, height - r.min.y));
		if let egui::epaint::Primitive::Mesh(m) = &mut p.primitive {
			for v in &mut m.vertices {
				v.pos.y = height - v.pos.y;
			}
		}
	}
}

/// Writes a picture as a PNG
pub fn save_png(image: &egui::ColorImage, path: &std::path::Path) -> Result<(), String> {
	let [w, h] = image.size;
	let bytes: Vec<u8> = image.pixels.iter().flat_map(|c| c.to_array()).collect();
	image::save_buffer(path, &bytes, w as u32, h as u32, image::ExtendedColorType::Rgba8).map_err(|e| e.to_string())
}
