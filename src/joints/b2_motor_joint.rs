use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_motor_joint as private;

impl<D: UserDataType> Default for B2motorJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EMotorJoint,
				..Default::default()
			},
			linear_offset: B2vec2::zero(),
			angular_offset: 0.0,
			max_force: 1.0,
			max_torque: 1.0,
			correction_factor: 0.3,
		};
	}
}

/// Motor joint definition.
#[derive(Clone)]
pub struct B2motorJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// Position of body_b minus the position of body_a, in body_a's frame, in meters.
	pub linear_offset: B2vec2,

	/// The body_b angle minus body_a angle in radians.
	pub angular_offset: f32,
	/// The maximum motor force in n.
	pub max_force: f32,

	/// The maximum motor torque in n-m.
	pub max_torque: f32,

	/// Position correction factor in the range [0,1].
	pub correction_factor: f32,
}

impl<D: UserDataType> B2motorJointDef<D> {
	/// initialize the bodies and offsets using the current transforms.
	//void initialize(b2_body* body_a, b2_body* body_b);
	pub fn initialize(&mut self, body_a: BodyPtr<D>, body_b: BodyPtr<D>) {
		self.base.body_a = Some(body_a.clone());
		self.base.body_b = Some(body_b.clone());

		let x_b: B2vec2 = body_b.borrow().get_position();
		self.linear_offset = body_a.borrow().get_local_point(x_b);

		let angle_a: f32 = body_a.borrow().get_angle();
		let angle_b: f32 = body_b.borrow().get_angle();
		self.angular_offset = angle_b - angle_a;
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2motorJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EMotorJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EMotorJoint(self);
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2motorJoint<D> {
	fn get_base(&self) -> &B2joint<D> {
		return &self.base;
	}
	fn get_base_mut(&mut self) -> &mut B2joint<D> {
		return &mut self.base;
	}
	fn get_anchor_a(&self) -> B2vec2 {
		return self.base.m_body_a.borrow().get_position();
	}
	fn get_anchor_b(&self) -> B2vec2 {
		return self.base.m_body_b.borrow().get_position();
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

	/// Dump to b2Log
	fn dump(&self) {
		private::dump(self);
	}
}

impl<D: UserDataType> B2motorJoint<D> {
	/// Set/get the target linear offset, in frame A, in meters.
	pub fn set_linear_offset(&mut self, linear_offset: B2vec2) {
		if linear_offset.x != self.m_linear_offset.x || linear_offset.y != self.m_linear_offset.y {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_linear_offset = linear_offset;
		}
	}
	pub fn get_linear_offset(&self) -> B2vec2 {
		return self.m_linear_offset;
	}

	/// Set/get the target angular offset, in radians.
	pub fn set_angular_offset(&mut self, angular_offset: f32) {
		if angular_offset != self.m_angular_offset {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_angular_offset = angular_offset;
		}
	}
	pub fn get_angular_offset(&self) -> f32 {
		return self.m_angular_offset;
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

	/// Set the position correction factor in the range [0,1].
	pub fn set_correction_factor(&mut self, factor: f32) {
		b2_assert(b2_is_valid(factor) && 0.0 <= factor && factor <= 1.0);
		self.m_correction_factor = factor;
	}

	/// Get the position correction factor in the range [0,1].
	pub fn get_correction_factor(&self) -> f32 {
		return self.m_correction_factor;
	}

	pub(crate) fn new(def: &B2motorJointDef<D>) -> Self {
		return Self {
			base: B2joint::new(&def.base),
			m_linear_offset: def.linear_offset,
			m_angular_offset: def.angular_offset,

			m_linear_impulse: B2vec2::zero(),
			m_angular_impulse: 0.0,

			m_max_force: def.max_force,
			m_max_torque: def.max_torque,
			m_correction_factor: def.correction_factor,

			m_index_a: 0,
			m_index_b: 0,
			m_r_a: B2vec2::zero(),
			m_r_b: B2vec2::zero(),
			m_local_center_a: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_linear_error: B2vec2::zero(),
			m_angular_error: 0.0,
			m_inv_mass_a: 0.0,
			m_inv_mass_b: 0.0,
			m_inv_ia: 0.0,
			m_inv_ib: 0.0,
			m_linear_mass: B2Mat22::new(B2vec2::zero(), B2vec2::zero()),
			m_angular_mass: 0.0,
		};
	}
}

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
pub struct B2motorJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,

	// Solver shared
	pub(crate) m_linear_offset: B2vec2,
	pub(crate) m_angular_offset: f32,
	pub(crate) m_linear_impulse: B2vec2,
	pub(crate) m_angular_impulse: f32,
	pub(crate) m_max_force: f32,
	pub(crate) m_max_torque: f32,
	pub(crate) m_correction_factor: f32,

	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_r_a: B2vec2,
	pub(crate) m_r_b: B2vec2,
	pub(crate) m_local_center_a: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_linear_error: B2vec2,
	pub(crate) m_angular_error: f32,
	pub(crate) m_inv_mass_a: f32,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ia: f32,
	pub(crate) m_inv_ib: f32,
	pub(crate) m_linear_mass: B2Mat22,
	pub(crate) m_angular_mass: f32,
}
