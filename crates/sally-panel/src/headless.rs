//! Rendering without a display, on Linux: an OpenGL 3.3 context on EGL's software device (Mesa's llvmpipe),
//! surfaceless, which `Offscreen` views draw on the way the window host draws the overlay. For the tests and the
//! screenshots (the C++ panel's main_headless.cpp, which did the same through Mesa's surfaceless platform).

use std::sync::Arc;

use glutin::api::egl::context::PossiblyCurrentContext;
use glutin::api::egl::device::Device;
use glutin::api::egl::display::Display;
use glutin::config::{ConfigSurfaceTypes, ConfigTemplateBuilder};
use glutin::context::{ContextApi, ContextAttributesBuilder, Version};
use glutin::prelude::*;

pub struct Headless {
	pub gl: Arc<glow::Context>,
	_context: PossiblyCurrentContext,
	_display: Display,
}

impl Headless {
	/// The GL context, current on this thread; Err when there is no EGL software device
	pub fn new() -> Result<Headless, String> {
		let devices: Vec<Device> = Device::query_devices().map_err(|e| format!("EGL devices: {e}"))?.collect();
		let device = devices
			.iter()
			.find(|d| d.extensions().contains("EGL_MESA_device_software"))
			.or(devices.first())
			.ok_or("no EGL device")?;
		// SAFETY: no raw display: the display is the device's own
		let display = unsafe { Display::with_device(device, None) }.map_err(|e| format!("EGL display: {e}"))?;
		let template = ConfigTemplateBuilder::new().with_surface_type(ConfigSurfaceTypes::empty()).build();
		// SAFETY: the template is a plain config request on this display
		let config = unsafe { display.find_configs(template) }
			.map_err(|e| format!("EGL configs: {e}"))?
			.next()
			.ok_or("no EGL config")?;
		let attrs =
			ContextAttributesBuilder::new().with_context_api(ContextApi::OpenGl(Some(Version::new(3, 3)))).build(None);
		// SAFETY: a context of this display's config, with no window
		let context = unsafe { display.create_context(&config, &attrs) }.map_err(|e| format!("EGL context: {e}"))?;
		let context = context.make_current_surfaceless().map_err(|e| format!("EGL make current: {e}"))?;
		// SAFETY: the loader gives the current context's functions
		let gl = unsafe { glow::Context::from_loader_function_cstr(|s| display.get_proc_address(s)) };
		Ok(Headless { gl: Arc::new(gl), _context: context, _display: display })
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::offscreen::Offscreen;

	#[test]
	fn a_view_draws_into_its_texture() {
		let h = match Headless::new() {
			Ok(h) => h,
			Err(e) => {
				eprintln!("no headless GL here ({e}): nothing drawn");
				return;
			}
		};
		let mut v = Offscreen::new(h.gl.clone(), 200, 100, 2.0).expect("view");
		assert_eq!(v.size_points(), egui::vec2(100.0, 50.0));
		for _ in 0..2 {
			v.frame(0.0, Vec::new(), |ui| {
				ui.painter().rect_filled(
					egui::Rect::from_min_size(egui::pos2(10.0, 10.0), egui::vec2(20.0, 10.0)),
					0.0,
					egui::Color32::RED,
				);
			});
		}
		let img = v.read();
		assert_eq!(img.size, [200, 100]);
		// the rect from (10, 10) to (30, 20) points is (20, 20) to (60, 40) pixels, top row first
		assert_eq!(img.pixels[30 * 200 + 40], egui::Color32::RED);
		assert_eq!(img.pixels[80 * 200 + 40], egui::Color32::TRANSPARENT);
		v.mirror = true;
		v.frame(0.0, Vec::new(), |ui| {
			ui.painter().rect_filled(
				egui::Rect::from_min_size(egui::pos2(10.0, 10.0), egui::vec2(20.0, 10.0)),
				0.0,
				egui::Color32::RED,
			);
		});
		let img = v.read();
		assert_eq!(img.pixels[30 * 200 + 40], egui::Color32::TRANSPARENT);
		assert_eq!(img.pixels[(99 - 30) * 200 + 40], egui::Color32::RED);
		v.destroy();
	}
}
