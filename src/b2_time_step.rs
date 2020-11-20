use crate::b2_math::*;

/// Profiling data. Times are in milliseconds.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2Profile {
	pub step: f32,
	pub collide: f32,
	pub solve: f32,
	pub solve_init: f32,
	pub solve_velocity: f32,
	pub solve_position: f32,
	pub broadphase: f32,
	pub solve_toi: f32,
}

/// This is an internal structure.
#[derive(Default, Clone, Copy, Debug)]
pub(crate) struct B2timeStep {
	pub dt: f32,      // time step
	pub inv_dt: f32,  // inverse time step (0 if dt == 0).
	pub dt_ratio: f32, // dt * inv_dt0
	pub velocity_iterations: i32,
	pub position_iterations: i32,
	pub warm_starting: bool,
}

/// This is an internal structure.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2position {
	pub(crate) c: B2vec2,
	pub(crate) a: f32,
}

/// This is an internal structure.
#[derive(Default, Clone, Copy, Debug)]
pub struct B2velocity {
	pub(crate) v: B2vec2,
	pub(crate) w: f32,
}

/// Solver Data
#[derive(Default, Debug)]
pub struct B2solverData {
	pub(crate) step: B2timeStep,
	//pub(crate) positions: &'a mut  [B2position],
	//pub(crate) velocities: &'a mut  [B2velocity],
}
