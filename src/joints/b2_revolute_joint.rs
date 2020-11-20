use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_revolute_joint as private;
use crate::b2_draw::*;

impl<D: UserDataType> Default for B2revoluteJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::ERevoluteJoint,
				..Default::default()
			},
			local_anchor_a: B2vec2::zero(),
			local_anchor_b: B2vec2::zero(),
			reference_angle: 0.0,
			lower_angle: 0.0,
			upper_angle: 0.0,
			max_motor_torque: 0.0,
			motor_speed: 0.0,
			enable_limit: false,
			enable_motor: false,
		};
	}
}

/// Revolute joint definition. This requires defining an anchor point where the
/// bodies are joined. The definition uses local anchor points so that the
/// initial configuration can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This helps when saving
/// and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
#[derive(Clone)]
pub struct B2revoluteJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The body_b angle minus body_a angle in the reference state (radians).
	pub reference_angle: f32,

	/// A flag to enable joint limits.
	pub enable_limit: bool,

	/// The lower angle for the joint limit (radians).
	pub lower_angle: f32,

	/// The upper angle for the joint limit (radians).
	pub upper_angle: f32,

	/// A flag to enable the joint motor.
	pub enable_motor: bool,

	/// The desired motor speed. Usually in radians per second.
	pub motor_speed: f32,

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in n-m.
	pub max_motor_torque: f32,
}

impl<D: UserDataType> B2revoluteJointDef<D> {
	/// initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	pub fn initialize(&mut self, body_a: BodyPtr<D>, body_b: BodyPtr<D>, anchor: B2vec2) {
		self.base.body_a = Some(body_a.clone());
		self.base.body_b = Some(body_b.clone());
		self.local_anchor_a = body_a.borrow().get_local_point(anchor);
		self.local_anchor_b = body_b.borrow().get_local_point(anchor);
		self.reference_angle = body_b.borrow().get_angle() - body_a.borrow().get_angle();
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2revoluteJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::ERevoluteJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::ERevoluteJoint(self);
	}
}

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
pub struct B2revoluteJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,
	// Solver shared
	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_impulse: B2vec2,
	pub(crate) m_motor_impulse: f32,
	pub(crate) m_lower_impulse: f32,
	pub(crate) m_upper_impulse: f32,

	pub(crate) m_enable_motor: bool,
	pub(crate) m_max_motor_torque: f32,
	pub(crate) m_motor_speed: f32,

	pub(crate) m_enable_limit: bool,
	pub(crate) m_reference_angle: f32,
	pub(crate) m_lower_angle: f32,
	pub(crate) m_upper_angle: f32,

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
	pub(crate) m_k: B2Mat22,
	pub(crate) m_angle: f32,
	pub(crate) m_axial_mass: f32,

}

impl<D: UserDataType> B2revoluteJoint<D> {
	/// The local anchor point relative to body_a's origin.
	pub fn get_local_anchor_a(&self) -> B2vec2 {
		return self.m_local_anchor_a;
	}

	/// The local anchor point relative to body_b's origin.
	pub fn get_local_anchor_b(&self) -> B2vec2 {
		return self.m_local_anchor_b;
	}

	/// Get the reference angle.
	pub fn get_reference_angle(&self) -> f32 {
		return self.m_reference_angle;
	}

	/// Get the current joint angle in radians.
	pub fn get_joint_angle(&self) -> f32 {
		let b_a = self.base.m_body_a.borrow();
		let b_b = self.base.m_body_b.borrow();
		return b_b.m_sweep.a - b_a.m_sweep.a - self.m_reference_angle;
	}

	/// Get the current joint angle speed in radians per second.
	pub fn get_joint_speed(&self) -> f32 {
		let b_a = self.base.m_body_a.borrow();
		let b_b = self.base.m_body_b.borrow();
		return b_b.m_angular_velocity - b_a.m_angular_velocity;
	}

	/// Is the joint limit enabled?
	pub fn is_limit_enabled(&self) -> bool {
		return self.m_enable_limit;
	}

	/// Enable/disable the joint limit.
	pub fn enable_limit(&mut self, flag: bool) {
		if flag != self.m_enable_limit {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_enable_limit = flag;
			self.m_lower_impulse = 0.0;
			self.m_upper_impulse = 0.0;
	
		}
	}

	/// Get the lower joint limit in radians.
	pub fn get_lower_limit(&self) -> f32 {
		return self.m_lower_angle;
	}

	/// Get the upper joint limit in radians.
	pub fn get_upper_limit(&self) -> f32 {
		return self.m_upper_angle;
	}

	/// Set the joint limits in radians.
	pub fn set_limits(&mut self, lower: f32, upper: f32) {
		b2_assert(lower <= upper);

		if lower != self.m_lower_angle || upper != self.m_upper_angle {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_lower_impulse = 0.0;
			self.m_upper_impulse = 0.0;
			self.m_lower_angle = lower;
			self.m_upper_angle = upper;
		}
	}

	/// Is the joint motor enabled?
	pub fn is_motor_enabled(&self) -> bool {
		return self.m_enable_motor;
	}

	/// Enable/disable the joint motor.
	pub fn enable_motor(&mut self, flag: bool) {
		if flag != self.m_enable_motor {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_enable_motor = flag;
		}
	}

	/// Set the motor speed in radians per second.
	pub fn set_motor_speed(&mut self, speed: f32) {
		if speed != self.m_motor_speed {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_motor_speed = speed;
		}
	}

	/// Get the motor speed in radians per second.
	pub fn get_motor_speed(&self) -> f32 {
		return self.m_motor_speed;
	}

	/// Set the maximum motor torque, usually in n-m.
	pub fn set_max_motor_torque(&mut self, torque: f32) {
		if torque != self.m_max_motor_torque {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_max_motor_torque = torque;
		}
	}
	pub fn get_max_motor_torque(&self) -> f32 {
		return self.m_max_motor_torque;
	}

	/// Get the current motor torque given the inverse time step.
	/// Unit is n*m.
	pub fn get_motor_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * self.m_motor_impulse;
	}

	pub(crate) fn new(def: &B2revoluteJointDef<D>) -> Self {
		return Self {
			base: B2joint::new(&def.base),

			m_local_anchor_a: def.local_anchor_a,
			m_local_anchor_b: def.local_anchor_b,
			m_impulse: B2vec2::zero(),
			m_axial_mass : 0.0,
			m_motor_impulse : 0.0,
			m_lower_impulse : 0.0,
			m_upper_impulse : 0.0,
		
			m_enable_motor: def.enable_motor,
			m_max_motor_torque: def.max_motor_torque,
			m_motor_speed: def.motor_speed,

			m_enable_limit: def.enable_limit,
			m_reference_angle: def.reference_angle,
			m_lower_angle: def.lower_angle,
			m_upper_angle: def.upper_angle,

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
			m_k: B2Mat22::zero(),
			m_angle: 0.0,
		}
	}
}

// inline f32 B2revoluteJoint::get_motor_speed() const
// {
// 	return m_motor_speed;
// }

impl<D: UserDataType> B2jointTraitDyn<D> for B2revoluteJoint<D> {
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
		let p = B2vec2::new(self.m_impulse.x, self.m_impulse.y);
		return inv_dt * p;
	}
	/// Get the reaction torque due to the joint limit given the inverse time step.
	/// Unit is n*m.
	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * (self.m_lower_impulse + self.m_upper_impulse);
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
		//private::dump(self);
	}

	fn draw(&self, draw: &mut dyn B2drawTrait) {
		private::draw(self, draw);
	} 
}
