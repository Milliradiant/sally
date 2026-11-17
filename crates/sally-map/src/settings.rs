//! The map solve's bound options: the STRUCT_CONFIG_ITEMs of driver_map_solve.c, with their C types. The C
//! binds each to a field of its map_solve struct (map_solve_attach_config), and the solve reads the fields where it
//! uses them, so a live "set" of one reaches the next read. Here the values come from a [`Source`]: the config store
//! as the map solve starts ([`Settings::read`]), or the variables the runtime binds to the options, read again into
//! the map solve whenever it is called ([`crate::MapSolve::set_settings`]). (The options are registered, with their
//! defaults, by the runtime. The file names and the test options are not bound: the map solve reads them from the
//! store where the C does, with survive_configs.)

use crate::host::Host;

/// What the bound options' values are read from, one option after the other in the C's order (each call names the
/// option and its C type: an int or a FLT)
pub trait Source {
	fn int(&mut self, tag: &'static str) -> i32;
	fn float(&mut self, tag: &'static str) -> f64;
}

/// The bound options, each with its tag
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
	/// map-solve-batch: most snapshots one solve takes (the oldest are dropped beyond this; at least 50 are kept)
	pub batch: i32,
	/// map-solve-step: at most one snapshot per device every this many s
	pub step: f64,
	/// map-solve-window: a snapshot's hits come from this many s before the pose (about one LH2 rotor turn)
	pub window: f64,
	/// map-solve-vmax, map-solve-wmax: snapshots only while the filter's speed (m/s) and turn rate (rad/s) are below
	pub vmax: f64,
	pub wmax: f64,
	/// map-solve-min-hits: hits a station must give a snapshot to count; a snapshot needs two such stations
	pub min_hits: i32,
	/// map-solve-min-snaps: snapshots a batch needs before it is solved
	pub min_snaps: i32,
	/// map-solve-min-station-hits: a station takes part in a solve with at least this many hits in the batch
	pub min_station_hits: i32,
	/// map-solve-first: no solve before this many s of run time
	pub first: f64,
	/// map-solve-period: s between joint solves
	pub period: f64,
	/// map-solve-tau: what earlier solves learned fades with this time constant (s)
	pub tau: f64,
	/// map-solve-iters: Gauss-Newton iterations per solve
	pub iters: i32,
	/// map-solve-prior-pos, map-solve-prior-rot: sd (m, rad) of the weak prior keeping each station near the
	/// calibrated map
	pub prior_pos: f64,
	pub prior_rot: f64,
	/// map-solve-rate-pos, map-solve-rate-rot: stations in use move toward the solution at most this fast (m/s, rad/s)
	pub rate_pos: f64,
	pub rate_rot: f64,
	/// map-solve-huber: robust (Huber) scale of a hit's angle residual (rad)
	pub huber: f64,
	/// map-solve-reject: hits further off than this (rad) are left out of a solve
	pub reject: f64,
	/// map-solve-max-move, map-solve-max-rot: a solve that moves a station further (m) or turns it further (rad) from
	/// the map in use is not applied
	pub max_move: f64,
	pub max_rot: f64,
	/// map-solve-load-weight: share of the saved map's information kept at the next start
	pub load_weight: f64,
	/// map-solve-save-period: also write the map file every this many s after an applied solve (<= 0: only at exit)
	pub save_period: f64,
	/// map-solve-relocate: handle a moved (bumped) station (0 off)
	pub relocate: i32,
	/// map-solve-move-resid: a station counts as moved when the median residual of its hits, from snapshot poses
	/// fitted to the other stations only, is above this (rad)
	pub move_resid: f64,
	/// map-solve-move-fit: a relocation counts only when it explains its station's light to this (rad)
	pub move_fit: f64,
	/// map-solve-move-ratio: ... and this many times the residual of the other stations in that fit
	pub move_ratio: f64,
	/// map-solve-move-rest-ratio: ... and leaving any other station out instead leaves the rest this many times worse
	pub move_rest_ratio: f64,
	/// map-solve-move-min-snaps: snapshots seen by a station and two others that the test and a relocation need
	pub move_min_snaps: i32,
	/// map-solve-move-agree-pos, map-solve-move-agree-rot: bound by the C and never read
	pub move_agree_pos: f64,
	pub move_agree_rot: f64,
	/// map-solve-move-min, map-solve-move-min-rot: a relocation smaller than this (m) and this (rad) is left to the
	/// normal solve
	pub move_min_pos: f64,
	pub move_min_rot: f64,
	/// map-solve-move-pending-max: a relocation waiting for its confirmation is dropped after this many s
	pub move_pending_max: f64,
	/// map-solve-threaded: the C's solve thread (live), which the runtime starts (`MapSolve::start_thread`) unless the
	/// playback factor is 0 or it runs on one thread; 0 solves inline
	pub threaded: i32,
}

/// The config store as map_solve_attach_config reads it: survive_configi / survive_configf with SC_GET and 0, which
/// the option's registered default replaces
struct Store<'a, H: Host + ?Sized>(&'a mut H);

impl<H: Host + ?Sized> Source for Store<'_, H> {
	fn int(&mut self, tag: &'static str) -> i32 {
		self.0.configi(tag, 0) as i32
	}
	fn float(&mut self, tag: &'static str) -> f64 {
		self.0.configf(tag, 0.0)
	}
}

impl Settings {
	/// Each bound option from `src`, in the C's order (map_solve_attach_config's, the STRUCT_CONFIG_ITEMs' order)
	pub fn read_from(src: &mut impl Source) -> Settings {
		// a struct expression evaluates its fields in the order written: the C's
		Settings {
			batch: src.int("map-solve-batch"),
			step: src.float("map-solve-step"),
			window: src.float("map-solve-window"),
			vmax: src.float("map-solve-vmax"),
			wmax: src.float("map-solve-wmax"),
			min_hits: src.int("map-solve-min-hits"),
			min_snaps: src.int("map-solve-min-snaps"),
			min_station_hits: src.int("map-solve-min-station-hits"),
			first: src.float("map-solve-first"),
			period: src.float("map-solve-period"),
			tau: src.float("map-solve-tau"),
			iters: src.int("map-solve-iters"),
			prior_pos: src.float("map-solve-prior-pos"),
			prior_rot: src.float("map-solve-prior-rot"),
			rate_pos: src.float("map-solve-rate-pos"),
			rate_rot: src.float("map-solve-rate-rot"),
			huber: src.float("map-solve-huber"),
			reject: src.float("map-solve-reject"),
			max_move: src.float("map-solve-max-move"),
			max_rot: src.float("map-solve-max-rot"),
			load_weight: src.float("map-solve-load-weight"),
			save_period: src.float("map-solve-save-period"),
			relocate: src.int("map-solve-relocate"),
			move_resid: src.float("map-solve-move-resid"),
			move_fit: src.float("map-solve-move-fit"),
			move_ratio: src.float("map-solve-move-ratio"),
			move_rest_ratio: src.float("map-solve-move-rest-ratio"),
			move_min_snaps: src.int("map-solve-move-min-snaps"),
			move_agree_pos: src.float("map-solve-move-agree-pos"),
			move_agree_rot: src.float("map-solve-move-agree-rot"),
			move_min_pos: src.float("map-solve-move-min"),
			move_min_rot: src.float("map-solve-move-min-rot"),
			move_pending_max: src.float("map-solve-move-pending-max"),
			threaded: src.int("map-solve-threaded"),
		}
	}

	/// map_solve_attach_config's reads from the config store (the values the variables take as they are bound)
	pub fn read(host: &mut (impl Host + ?Sized)) -> Settings {
		Settings::read_from(&mut Store(host))
	}
}
