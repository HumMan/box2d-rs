use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;

use crate::private::dynamics::joints::b2_distance_joint as private;

impl<D: UserDataType> Default for B2distanceJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EDistanceJoint,
				..Default::default()
			},
			local_anchor_a: B2vec2::zero(),
			local_anchor_b: B2vec2::zero(),
			length: 1.0,
			stiffness: 0.0,
			damping: 0.0,
		};
	}
}

/// Distance joint definition. This requires defining an anchor point on both
/// bodies and the non-zero length of the distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
#[derive(Clone)]
pub struct B2distanceJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The natural length between the anchor points.
	pub length: f32,

	/// The linear stiffness in n/m. A value of 0 disables softness.
	pub stiffness: f32,

	/// The linear damping in n*s/m.
	pub damping: f32,

}

impl<D: UserDataType> B2distanceJointDef<D> {
	/// initialize the bodies, anchors, and length using the world
	/// anchors.
	pub fn initialize(
		&mut self,
		body_a: BodyPtr<D>,
		body_b: BodyPtr<D>,
		anchor_a: B2vec2,
		anchor_b: B2vec2,
	) {
		private::b2_distance_joint_def_initialize(self, body_a, body_b, anchor_a, anchor_b);
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2distanceJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EDistanceJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EDistanceJoint(self);
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2distanceJoint<D> {
	fn get_base(&self) -> &B2joint<D> {
		return &self.base;
	}
	fn get_base_mut(&mut self) -> &mut B2joint<D> {
		return &mut self.base;
	}
	fn get_anchor_a(&self) -> B2vec2 {
		return private::get_anchor_a(self);
	}
	fn get_anchor_b(&self) -> B2vec2 {
		return private::get_anchor_b(self);
	}

	/// Get the reaction force given the inverse time step.
	/// Unit is n.
	fn get_reaction_force(&self, inv_dt: f32) -> B2vec2 {
		return private::get_reaction_force(self, inv_dt);
	}

	/// Get the reaction torque given the inverse time step.
	/// Unit is n*m. This is always zero for a distance joint.
	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		return private::get_reaction_torque(self, inv_dt);
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
}

impl<D: UserDataType> B2distanceJoint<D> {
	/// The local anchor point relative to body_a's origin.
	pub fn get_local_anchor_a(&self) -> B2vec2 {
		return self.m_local_anchor_a;
	}

	/// The local anchor point relative to body_b's origin.
	pub fn get_local_anchor_b(&self) -> B2vec2 {
		return self.m_local_anchor_b;
	}

	/// Set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	pub fn set_length(&mut self, length: f32) {
		self.m_length = length;
	}
	pub fn get_length(&self) -> f32 {
		return self.m_length;
	}

	/// Set/get the linear stiffness in n/m
	pub fn set_stiffness(&mut self, stiffness: f32) {
		self.m_stiffness = stiffness;
	}
	pub fn get_stiffness(&self) -> f32 {
		return self.m_stiffness;
	}

	/// Set/get linear damping in n*s/m
	pub fn set_damping(&mut self, damping: f32) {
		self.m_damping = damping;
	}
	pub fn get_damping(&self) -> f32 {
		return self.m_damping;
	}

	pub(crate) fn new(data: &B2distanceJointDef<D>) -> Self {
		return private::b2_distance_joint_new(data);
	}
}

/// A distance joint constrains two points on two bodies to remain at a fixed
/// distance from each other. You can view this as a massless, rigid rod.
pub struct B2distanceJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,

	// protected:
	pub(crate) m_stiffness: f32,
	pub(crate) m_damping: f32,
	pub(crate) m_bias: f32,

	// Solver shared
	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_gamma: f32,
	pub(crate) m_impulse: f32,
	pub(crate) m_length: f32,

	// Solver temp
	pub(crate) m_index_a: usize,
	pub(crate) m_index_b: usize,
	pub(crate) m_u: B2vec2,
	pub(crate) m_r_a: B2vec2,
	pub(crate) m_r_b: B2vec2,
	pub(crate) m_local_center_a: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_inv_mass_a: f32,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ia: f32,
	pub(crate) m_inv_ib: f32,
	pub(crate) m_mass: f32,
}
