//! The IMU samples of a packed recording. An accelerometer or gyro axis is its last value plus what a small adaptive
//! filter makes of its last changes, and the sample is coded as what is left: a few bits of sensor noise while the
//! device is still, more as it moves. The filter is integers only, so the packer and the unpacker compute the same
//! prediction on any machine.

use sally_pack::coder::{Io, Number};

/// How many past changes the filter weighs
const ORDER: usize = 4;
/// The weights' fixed point (1.0 is `1 << WEIGHT`)
const WEIGHT: u32 = 16;
/// The filter's step, as a shift: each sample moves the weights 1/32 of the way to predicting it exactly
const STEP: u32 = 5;
/// The contexts a residual's length is coded under, per axis: the length of that axis's residual before it
const LENGTHS: usize = 18;

/// One axis: its last value, its last changes and the weights that predict the next change from them (a normalised
/// LMS filter)
#[derive(Clone, Default)]
struct Axis {
	last: i32,
	changes: [i32; ORDER],
	weights: [i32; ORDER],
	/// the length of the last residual, the next one's context
	length: usize,
}

impl Axis {
	fn filtered(&self) -> i32 {
		let sum: i64 = self.changes.iter().zip(&self.weights).map(|(&c, &w)| c as i64 * w as i64).sum();
		((sum + (1 << (WEIGHT - 1))) >> WEIGHT) as i32
	}

	fn predict(&self) -> i32 {
		self.last + self.filtered()
	}

	fn update(&mut self, value: i32) {
		let change = value - self.last;
		let error = (change - self.filtered()) as i64;
		// the step is the error over the changes' energy: large changes do not swing the weights
		let energy: i64 = self.changes.iter().map(|&c| c as i64 * c as i64).sum::<i64>() + 16;
		let step = (error << WEIGHT) / energy;
		for (w, &c) in self.weights.iter_mut().zip(&self.changes) {
			let moved = *w as i64 + ((step * c as i64) >> STEP);
			*w = moved.clamp(-(4 << WEIGHT), 4 << WEIGHT) as i32;
		}
		self.changes.copy_within(0..ORDER - 1, 1);
		self.changes[0] = change;
		self.last = value;
	}
}

/// A device's six axes (three of the accelerometer, three of the gyro)
#[derive(Clone, Default)]
pub struct Imu {
	axes: [Axis; 6],
}

/// The models the residuals are coded under: one per kind of device report, shared by the devices that send it
pub struct ImuModels {
	residual: Number,
}

impl ImuModels {
	pub fn new() -> ImuModels {
		ImuModels { residual: Number::new(6 * LENGTHS) }
	}
}

impl Imu {
	/// One axis's sample (a 16-bit value)
	pub fn code<I: Io>(&mut self, io: &mut I, m: &mut ImuModels, axis: usize, value: i16) -> i16 {
		let a = &mut self.axes[axis];
		let predicted = a.predict();
		let residual = m.residual.code(io, axis * LENGTHS + a.length, (value as i32 - predicted) as i64);
		// packing, this is the value itself; unpacking a damaged block it may be anything: it is cut to 16 bits either
		// way, the same on both sides
		let value = (predicted as i64 + residual) as i16;
		a.length = Number::length_of(residual).min(LENGTHS - 1);
		a.update(value as i32);
		value
	}
}
