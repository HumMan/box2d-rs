use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_rope_joint as private;

impl<D: UserDataType> Default for B2ropeJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::ERopeJoint,
				..Default::default()
			},
			local_anchor_a: B2vec2::new(-1.0, 0.0),
			local_anchor_b: B2vec2::new(1.0, 0.0),
			max_length: 0.0,
		};
	}
}

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collide_connected in B2jointDef.
#[derive(Clone)]
pub struct B2ropeJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The maximum length of the rope.
	/// Warning: this must be larger than B2_LINEAR_SLOP or
	/// the joint will have no effect.
	pub max_length: f32,
}

impl<D: UserDataType> ToDerivedJoint<D> for B2ropeJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::ERopeJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::ERopeJoint(self);
	}
}

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so i chose not to implement it
/// that way. See B2distanceJoint if you want to dynamically
/// control length.
pub struct B2ropeJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,

	// Solver shared
	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_max_length: f32,
	pub(crate) m_length: f32,
	pub(crate) m_impulse: f32,

	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
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

impl<D: UserDataType> B2ropeJoint<D> {
	/// The local anchor point relative to body_a's origin.
	pub fn get_local_anchor_a(&self) -> B2vec2 {
		return self.m_local_anchor_a;
	}

	/// The local anchor point relative to body_b's origin.
	pub fn get_local_anchor_b(&self) -> B2vec2 {
		return self.m_local_anchor_b;
	}

	/// Set/Get the maximum length of the rope.
	pub fn set_max_length(&mut self, length: f32) {
		self.m_max_length = length;
	}
	pub fn get_max_length(&self) -> f32 {
		return self.m_max_length;
	}

	// Get current length
	pub fn get_length(&self) -> f32 {
		return self.m_length;
	}

	pub(crate) fn new(def: &B2ropeJointDef<D>) -> Self {
		return Self {
			base: B2joint::new(&def.base),

			m_local_anchor_a: def.local_anchor_a,
			m_local_anchor_b: def.local_anchor_b,

			m_max_length: def.max_length,
			m_length: 0.0,
			m_impulse: 0.0,

			m_index_a: 0,
			m_index_b: 0,
			m_u: B2vec2::zero(),
			m_r_a: B2vec2::zero(),
			m_r_b: B2vec2::zero(),
			m_local_center_a: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_inv_mass_a: 0.0,
			m_inv_mass_b: 0.0,
			m_inv_ia: 0.0,
			m_inv_ib: 0.0,
			m_mass: 0.0,
		};
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2ropeJoint<D> {
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
		let f: B2vec2 = (inv_dt * self.m_impulse) * self.m_u;
		return f;
	}

	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		b2_not_used(inv_dt);
		return 0.0;
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
