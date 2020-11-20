use crate::b2_distance::B2distanceProxy;
use crate::b2_math::B2Sweep;
use crate::private::collision::b2_time_of_impact as private;

use std::sync::atomic::{AtomicUsize,AtomicU64};

/// ns
pub static B2_TOI_TIME: AtomicU64 = AtomicU64::new(0);
/// ns
pub static B2_TOI_MAX_TIME: AtomicU64 = AtomicU64::new(0);

pub static B2_TOI_CALLS: AtomicUsize = AtomicUsize::new(0);
pub static B2_TOI_ITERS: AtomicUsize = AtomicUsize::new(0);
pub static B2_TOI_MAX_ITERS: AtomicUsize = AtomicUsize::new(0);
pub static B2_TOI_ROOT_ITERS: AtomicUsize = AtomicUsize::new(0);
pub static B2_TOI_MAX_ROOT_ITERS: AtomicUsize = AtomicUsize::new(0);

/// Input parameters for b2TimeOfImpact
#[derive(Default, Clone, Debug)]
pub struct B2toiinput {
	pub proxy_a: B2distanceProxy,
	pub proxy_b: B2distanceProxy,
	pub sweep_a: B2Sweep,
	pub sweep_b: B2Sweep,
	pub t_max: f32, // defines sweep interval [0, t_max]
}

#[derive(Clone, Debug, PartialEq)]
pub enum B2toioutputState {
	EUnknown,
	EFailed,
	EOverlapped,
	ETouching,
	ESeparated,
}

impl Default for B2toioutputState
{
	fn default()->Self
	{
		return B2toioutputState::EUnknown;
	}
}

/// Output parameters for b2TimeOfImpact.
#[derive(Default, Clone, Debug)]
pub struct B2toioutput {
	pub state: B2toioutputState,
	pub t: f32,
}

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,t_max]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
/// Note: use b2Distance to compute the contact point and normal at the time of impact.
pub fn b2_time_of_impact(output: &mut B2toioutput, input: &B2toiinput) {
	private::b2_time_of_impact(output, input);
}
