use std::time::Instant;

pub struct B2timer {
	start: Instant,
}

impl Default for B2timer
{
	/// Constructor
	fn default() -> Self {
		B2timer { start: Instant::now() }
	}
}

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
impl B2timer {

	/// reset the timer.
	pub fn reset(&mut self) {
		self.start = Instant::now();
	}

	/// Get the time since construction or the last reset.
	pub fn get_milliseconds(&self) -> f32 {
		let elapsed = self.start.elapsed();
		let nanos = elapsed.subsec_nanos() as u64;
		let ms = 1000.0 * elapsed.as_secs() as f32 + nanos as f32/(1000.0 * 1000.0);
		ms as f32
	}
	pub fn precise_time_ns(&self) -> u64 {
		let elapsed = self.start.elapsed();
		let nanos = elapsed.subsec_nanos() as u64;
		let ns = 1000*1000*1000 * elapsed.as_secs() + nanos;
		ns
	}
}
