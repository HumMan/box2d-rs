use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_gear_joint as private;

impl<D: UserDataType> Default for B2gearJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EGearJoint,
				..Default::default()
			},
			joint1: None,
			joint2: None,
			ratio: 1.0,
		};
	}
}

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
#[derive(Clone)]
pub struct B2gearJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,
	/// The first revolute/prismatic joint attached to the gear joint.
	pub joint1: Option<B2jointPtr<D>>,

	/// The second revolute/prismatic joint attached to the gear joint.
	pub joint2: Option<B2jointPtr<D>>,

	/// The gear ratio.
	/// [see](B2gearJoint) for explanation.
	pub ratio: f32,
}

impl<D: UserDataType> ToDerivedJoint<D> for B2gearJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EGearJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EGearJoint(self);
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2gearJoint<D> {
	fn get_base(&self) -> &B2joint<D> {
		return &self.base;
	}
	fn get_base_mut(&mut self) -> &mut B2joint<D> {
		return &mut self.base;
	}
	fn get_anchor_a(&self) -> B2vec2 {
		return self
			.base
			.m_body_a
			.borrow()
			.get_world_point(self.m_local_anchor_a);
	}
	fn get_anchor_b(&self) -> B2vec2 {
		return self
			.base
			.m_body_b
			.borrow()
			.get_world_point(self.m_local_anchor_b);
	}

	/// Get the reaction force given the inverse time step.
	/// Unit is n.
	fn get_reaction_force(&self, inv_dt: f32) -> B2vec2 {
		let p: B2vec2 = self.m_impulse * self.m_jv_ac;
		return inv_dt * p;
	}

	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		let l: f32 = self.m_impulse * self.m_jw_a;
		return inv_dt * l;
	}
	fn init_velocity_constraints(
		&mut self,
		data: &mut B2solverData,
		positions: &mut [B2position],
		velocities: &mut [B2velocity],
	) {
		private::init_velocity_constraints(self, data, positions, velocities);
	}
	fn solve_velocity_constraints(
		&mut self,
		data: &mut B2solverData,
		velocities: &mut [B2velocity],
	) {
		private::solve_velocity_constraints(self, data, velocities);
	}
	fn solve_position_constraints(
		&mut self,
		data: &mut B2solverData,
		positions: &mut [B2position],
	) -> bool {
		return private::solve_position_constraints(self, data, positions);
	}

	/// dump joint to dmLog
	fn dump(&self) {
		private::dump(self);
	}
}

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
impl<D: UserDataType> B2gearJoint<D> {
	/// Get the first joint.
	pub fn get_joint1(&self) -> B2jointPtr<D> {
		return self.m_joint1.clone();
	}

	/// Get the second joint.
	pub fn get_joint2(&self) -> B2jointPtr<D> {
		return self.m_joint2.clone();
	}

	/// Set/Get the gear ratio.
	pub fn set_ratio(&mut self, ratio: f32) {
		b2_assert(b2_is_valid(ratio));
		self.m_ratio = ratio;
	}
	pub fn get_ratio(&self) -> f32 {
		return self.m_ratio;
	}

	pub(crate) fn new(data: &B2gearJointDef<D>)->Self
	{
		return private::new(data);
	}
}

pub struct B2gearJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,

	pub(crate) m_joint1: B2jointPtr<D>,
	pub(crate) m_joint2: B2jointPtr<D>,

	pub(crate) m_type_a: B2jointType,
	pub(crate) m_type_b: B2jointType,

	// Body A is connected to body c
	// Body b is connected to body D
	pub(crate) m_body_c: BodyPtr<D>,
	pub(crate) m_body_d: BodyPtr<D>,

	// Solver shared
	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_local_anchor_c: B2vec2,
	pub(crate) m_local_anchor_d: B2vec2,

	pub(crate) m_local_axis_c: B2vec2,
	pub(crate) m_local_axis_d: B2vec2,

	pub(crate) m_reference_angle_a: f32,
	pub(crate) m_reference_angle_b: f32,

	pub(crate) m_constant: f32,
	pub(crate) m_ratio: f32,

	pub(crate) m_impulse: f32,

	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_index_c: i32,
	pub(crate) m_index_d: i32,
	pub(crate) m_lc_a: B2vec2,
	pub(crate) m_lc_b: B2vec2,
	pub(crate) m_lc_c: B2vec2,
	pub(crate) m_lc_d: B2vec2,
	pub(crate) m_m_a: f32,
	pub(crate) m_m_b: f32,
	pub(crate) m_m_c: f32,
	pub(crate) m_m_d: f32,
	pub(crate) m_i_a: f32,
	pub(crate) m_i_b: f32,
	pub(crate) m_i_c: f32,
	pub(crate) m_i_d: f32,
	pub(crate) m_jv_ac: B2vec2,
	pub(crate) m_jv_bd: B2vec2,
	pub(crate) m_jw_a: f32,
	pub(crate) m_jw_b: f32,
	pub(crate) m_jw_c: f32,
	pub(crate) m_jw_d: f32,
	pub(crate) m_mass: f32,
}
