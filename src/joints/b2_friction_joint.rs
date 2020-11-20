use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_friction_joint as private;

impl<D: UserDataType> Default for B2frictionJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EFrictionJoint,
				..Default::default()
			},
			local_anchor_a: B2vec2::zero(),
			local_anchor_b: B2vec2::zero(),
			max_force: 0.0,
			max_torque: 0.0,
		};
	}
}

/// Friction joint definition.
#[derive(Clone)]
pub struct B2frictionJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,
	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The maximum friction force in n.
	pub max_force: f32,

	/// The maximum friction torque in n-m.
	pub max_torque: f32,
}

impl<D: UserDataType> B2frictionJointDef<D> {
	/// initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	pub fn initialize(&mut self, body_a: BodyPtr<D>, body_b: BodyPtr<D>, anchor: B2vec2) {
		self.base.body_a = Some(body_a.clone());
		self.base.body_b = Some(body_b.clone());
		self.local_anchor_a = body_a.borrow().get_local_point(anchor);
		self.local_anchor_b = body_b.borrow().get_local_point(anchor);
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2frictionJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EFrictionJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EFrictionJoint(self);
	}
}

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
pub struct B2frictionJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,

	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,

	// Solver shared
	pub(crate) m_linear_impulse: B2vec2,
	pub(crate) m_angular_impulse: f32,
	pub(crate) m_max_force: f32,
	pub(crate) m_max_torque: f32,
	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_r_a: B2vec2,
	pub(crate) m_r_b: B2vec2,
	pub(crate) m_local_center_a: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_inv_mass_a: f32,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ia: f32,
	pub(crate) m_inv_ib: f32,
	pub(crate) m_linear_mass: B2Mat22,
	pub(crate) m_angular_mass: f32,
}

impl<D: UserDataType> B2frictionJoint<D> {
	/// The local anchor point relative to body_a's origin.
	pub fn get_local_anchor_a(&self) -> B2vec2 {
		return self.m_local_anchor_a;
	}
	/// The local anchor point relative to body_b's origin.
	pub fn get_local_anchor_b(&self) -> B2vec2 {
		return self.m_local_anchor_b;
	}
	/// Set the maximum friction force in n.
	pub fn set_max_force(&mut self, force: f32) {
		b2_assert(b2_is_valid(force) && force >= 0.0);
		self.m_max_force = force;
	}

	/// Get the maximum friction force in n.
	pub fn get_max_force(&self) -> f32 {
		return self.m_max_force;
	}

	/// Set the maximum friction torque in n*m.
	pub fn set_max_torque(&mut self, torque: f32) {
		b2_assert(b2_is_valid(torque) && torque >= 0.0);
		self.m_max_torque = torque;
	}

	/// Get the maximum friction torque in n*m.
	pub fn get_max_torque(&self) -> f32 {
		return self.m_max_torque;
	}

	pub(crate) fn new(def: &B2frictionJointDef<D>) -> Self {
		return Self {
			base: B2joint::new(&def.base),
			m_local_anchor_a: def.local_anchor_a,
			m_local_anchor_b: def.local_anchor_b,

			m_linear_impulse: B2vec2::zero(),
			m_angular_impulse: 0.0,

			m_max_force: def.max_force,
			m_max_torque: def.max_torque,

			m_index_a: 0,
			m_index_b: 0,
			m_r_a: B2vec2::zero(),
			m_r_b: B2vec2::zero(),
			m_local_center_a: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_inv_mass_a: 0.0,
			m_inv_mass_b: 0.0,
			m_inv_ia: 0.0,
			m_inv_ib: 0.0,
			m_linear_mass: B2Mat22::new(B2vec2::zero(), B2vec2::zero()),
			m_angular_mass: 0.0,
		};
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2frictionJoint<D> {
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
		return inv_dt * self.m_linear_impulse;
	}

	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * self.m_angular_impulse;
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
