use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_draw::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_wheel_joint as private;

impl<D: UserDataType> Default for B2wheelJointDef<D> {
	fn default() -> Self {
		return Self {
			base: B2jointDef {
				jtype: B2jointType::EWheelJoint,
				..Default::default()
			},
			local_anchor_a: B2vec2::zero(),
			local_anchor_b: B2vec2::zero(),
			local_axis_a: B2vec2::new(1.0, 0.0),
			enable_limit: false,
			lower_translation: 0.0,
			upper_translation: 0.0,
			enable_motor: false,
			max_motor_torque: 0.0,
			motor_speed: 0.0,
			stiffness: 0.0,
			damping: 0.0,
		};
	}
}

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
#[derive(Clone)]
pub struct B2wheelJointDef<D: UserDataType> {
	pub base: B2jointDef<D>,

	/// The local anchor point relative to body_a's origin.
	pub local_anchor_a: B2vec2,

	/// The local anchor point relative to body_b's origin.
	pub local_anchor_b: B2vec2,

	/// The local translation axis in body_a.
	pub local_axis_a: B2vec2,

	/// Enable/disable the joint limit.
	pub enable_limit: bool,

	/// The lower translation limit, usually in meters.
	pub lower_translation: f32,

	/// The upper translation limit, usually in meters.
	pub upper_translation: f32,

	/// Enable/disable the joint motor.
	pub enable_motor: bool,

	/// The maximum motor torque, usually in n-m.
	pub max_motor_torque: f32,

	/// The desired motor speed in radians per second.
	pub motor_speed: f32,

	/// Suspension stiffness. Typically in units n/m.
	pub stiffness: f32,

	/// Suspension damping. Typically in units of n*s/m.
	pub damping: f32,
}

impl<D: UserDataType> B2wheelJointDef<D> {
	/// initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	pub fn initialize(
		&mut self,
		body_a: BodyPtr<D>,
		body_b: BodyPtr<D>,
		anchor: B2vec2,
		axis: B2vec2,
	) {
		self.base.body_a = Some(body_a.clone());
		self.base.body_b = Some(body_b.clone());
		self.local_anchor_a = body_a.borrow().get_local_point(anchor);
		self.local_anchor_b = body_b.borrow().get_local_point(anchor);
		self.local_axis_a = body_a.borrow().get_local_vector(axis);
	}
}

impl<D: UserDataType> ToDerivedJoint<D> for B2wheelJoint<D> {
	fn as_derived(&self) -> JointAsDerived<D> {
		return JointAsDerived::EWheelJoint(self);
	}
	fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
		return JointAsDerivedMut::EWheelJoint(self);
	}
}

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in body_a and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper. The spring/damper is
/// initialized upon creation. This joint is designed for vehicle suspensions.
pub struct B2wheelJoint<D: UserDataType> {
	pub(crate) base: B2joint<D>,

	pub(crate) m_local_anchor_a: B2vec2,
	pub(crate) m_local_anchor_b: B2vec2,
	pub(crate) m_local_xaxis_a: B2vec2,
	pub(crate) m_local_yaxis_a: B2vec2,

	pub(crate) m_impulse: f32,
	pub(crate) m_motor_impulse: f32,
	pub(crate) m_spring_impulse: f32,

	pub(crate) m_lower_impulse: f32,
	pub(crate) m_upper_impulse: f32,
	pub(crate) m_translation: f32,
	pub(crate) m_lower_translation: f32,
	pub(crate) m_upper_translation: f32,

	pub(crate) m_max_motor_torque: f32,
	pub(crate) m_motor_speed: f32,

	pub(crate) m_enable_limit: bool,
	pub(crate) m_enable_motor: bool,

	pub(crate) m_stiffness: f32,
	pub(crate) m_damping: f32,

	// Solver temp
	pub(crate) m_index_a: i32,
	pub(crate) m_index_b: i32,
	pub(crate) m_local_center_a: B2vec2,
	pub(crate) m_local_center_b: B2vec2,
	pub(crate) m_inv_mass_a: f32,
	pub(crate) m_inv_mass_b: f32,
	pub(crate) m_inv_ia: f32,
	pub(crate) m_inv_ib: f32,

	pub(crate) m_ax: B2vec2,
	pub(crate) m_ay: B2vec2,
	pub(crate) m_s_ax: f32,
	pub(crate) m_s_bx: f32,
	pub(crate) m_s_ay: f32,
	pub(crate) m_s_by: f32,

	pub(crate) m_mass: f32,
	pub(crate) m_motor_mass: f32,
	pub(crate) m_axial_mass: f32,
	pub(crate) m_spring_mass: f32,

	pub(crate) m_bias: f32,
	pub(crate) m_gamma: f32,
}

impl<D: UserDataType> B2wheelJoint<D> {
	/// The local anchor point relative to body_a's origin.
	pub fn get_local_anchor_a(&self) -> B2vec2 {
		return self.m_local_anchor_a;
	}

	/// The local anchor point relative to body_b's origin.
	pub fn get_local_anchor_b(&self) -> B2vec2 {
		return self.m_local_anchor_b;
	}

	/// The local joint axis relative to body_a.
	pub fn get_local_axis_a(&self) -> B2vec2 {
		return self.m_local_xaxis_a;
	}

	/// Get the current joint translation, usually in meters.
	pub fn get_joint_translation(&self) -> f32 {
		let b_a = self.base.m_body_a.borrow();
		let b_b = self.base.m_body_b.borrow();
		let p_a: B2vec2 = b_a.get_world_point(self.m_local_anchor_a);
		let p_b: B2vec2 = b_b.get_world_point(self.m_local_anchor_b);
		let d: B2vec2 = p_b - p_a;
		let axis: B2vec2 = b_a.get_world_vector(self.m_local_xaxis_a);
		let translation: f32 = b2_dot(d, axis);
		return translation;
	}

	/// Get the current joint linear speed, usually in meters per second.
	pub fn get_joint_linear_speed(&self) -> f32 {
		let b_a = self.base.m_body_a.borrow();
		let b_b = self.base.m_body_b.borrow();

		let r_a: B2vec2 =
			b2_mul_rot_by_vec2(b_a.m_xf.q, self.m_local_anchor_a - b_a.m_sweep.local_center);
		let r_b: B2vec2 =
			b2_mul_rot_by_vec2(b_b.m_xf.q, self.m_local_anchor_b - b_b.m_sweep.local_center);
		let p1: B2vec2 = b_a.m_sweep.c + r_a;
		let p2: B2vec2 = b_b.m_sweep.c + r_b;
		let d: B2vec2 = p2 - p1;
		let axis: B2vec2 = b2_mul_rot_by_vec2(b_a.m_xf.q, self.m_local_xaxis_a);

		let v_a: B2vec2 = b_a.m_linear_velocity;
		let v_b: B2vec2 = b_b.m_linear_velocity;
		let w_a: f32 = b_a.m_angular_velocity;
		let w_b: f32 = b_b.m_angular_velocity;

		let speed: f32 = b2_dot(d, b2_cross_scalar_by_vec(w_a, axis))
			+ b2_dot(
				axis,
				v_b + b2_cross_scalar_by_vec(w_b, r_b) - v_a - b2_cross_scalar_by_vec(w_a, r_a),
			);
		return speed;
	}

	/// Get the current joint angle in radians.
	pub fn get_joint_angle(&self) -> f32 {
		let b_a = self.base.m_body_a.borrow();
		let b_b = self.base.m_body_b.borrow();
		return b_b.m_sweep.a - b_a.m_sweep.a;
	}

	/// Get the current joint angular speed in radians per second.
	pub fn get_joint_angular_speed(&self) -> f32 {
		let w_a: f32 = self.base.m_body_a.borrow().m_angular_velocity;
		let w_b: f32 = self.base.m_body_b.borrow().m_angular_velocity;
		return w_b - w_a;
	}

	/// Is the joint limit enabled?
	pub fn is_limit_enabled(&self) -> bool {
		return self.m_enable_limit;
	}

	/// Enable/disable the joint translation limit.
	pub fn enable_limit(&mut self, flag: bool) {
		if flag != self.m_enable_limit {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_enable_limit = flag;
			self.m_lower_impulse = 0.0;
			self.m_upper_impulse = 0.0;
		}
	}

	/// Get the lower joint translation limit, usually in meters.
	pub fn get_lower_limit(&self) -> f32 {
		return self.m_lower_translation;
	}

	/// Get the upper joint translation limit, usually in meters.
	pub fn get_upper_limit(&self) -> f32 {
		return self.m_upper_translation;
	}

	/// Set the joint translation limits, usually in meters.
	pub fn set_limits(&mut self, lower: f32, upper: f32) {
		b2_assert(lower <= upper);
		if lower != self.m_lower_translation || upper != self.m_upper_translation {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_lower_translation = lower;
			self.m_upper_translation = upper;
			self.m_lower_impulse = 0.0;
			self.m_upper_impulse = 0.0;
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

	/// Set the motor speed, usually in radians per second.
	pub fn set_motor_speed(&mut self, speed: f32) {
		if speed != self.m_motor_speed {
			self.base.m_body_a.borrow_mut().set_awake(true);
			self.base.m_body_b.borrow_mut().set_awake(true);
			self.m_motor_speed = speed;
		}
	}

	/// Get the motor speed, usually in radians per second.
	pub fn get_motor_speed(&self) -> f32 {
		return self.m_motor_speed;
	}

	/// Set/Get the maximum motor force, usually in n-m.
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

	/// Get the current motor torque given the inverse time step, usually in n-m.
	pub fn get_motor_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * self.m_motor_impulse;
	}

	/// Access spring stiffness
	pub fn set_stiffness(&mut self, stiffness: f32) {
		self.m_stiffness = stiffness;
	}
	pub fn get_stiffness(&self) -> f32 {
		return self.m_stiffness;
	}

	/// Access damping
	pub fn set_damping(&mut self, damping: f32) {
		self.m_damping = damping;
	}
	pub fn get_damping(&self) -> f32 {
		return self.m_damping;
	}

	pub(crate) fn new(def: &B2wheelJointDef<D>) -> Self {
		return Self {
			base: B2joint::new(&def.base),
			m_local_anchor_a: def.local_anchor_a,
			m_local_anchor_b: def.local_anchor_b,
			m_local_xaxis_a: def.local_axis_a,
			m_local_yaxis_a: b2_cross_scalar_by_vec(1.0, def.local_axis_a),
			m_translation: 0.0,

			m_mass : 0.0,
			m_impulse : 0.0,
			m_motor_mass : 0.0,
			m_motor_impulse : 0.0,
			m_spring_mass : 0.0,
			m_spring_impulse : 0.0,

			m_axial_mass : 0.0,
			m_lower_impulse : 0.0,
			m_upper_impulse : 0.0,
			m_lower_translation : def.lower_translation,
			m_upper_translation : def.upper_translation,
			m_enable_limit : def.enable_limit,

			m_max_motor_torque : def.max_motor_torque,
			m_motor_speed : def.motor_speed,
			m_enable_motor : def.enable_motor,

			m_bias : 0.0,
			m_gamma : 0.0,

			m_ax: B2vec2::zero(),
			m_ay: B2vec2::zero(),

			m_stiffness : def.stiffness,
			m_damping : def.damping,

			// Solver temp
			m_index_a: 0,
			m_index_b: 0,
			m_local_center_a: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_inv_mass_a: 0.0,
			m_inv_mass_b: 0.0,
			m_inv_ia: 0.0,
			m_inv_ib: 0.0,

			m_s_ax: 0.0,
			m_s_bx: 0.0,
			m_s_ay: 0.0,
			m_s_by: 0.0,
		};
	}
}

impl<D: UserDataType> B2jointTraitDyn<D> for B2wheelJoint<D> {
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
		return inv_dt * (self.m_impulse * self.m_ay + (self.m_spring_impulse + self.m_lower_impulse - self.m_upper_impulse) * self.m_ax);
	}

	fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
		return inv_dt * self.m_motor_impulse;
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

	/// Debug draw this joint
	fn draw(&self, draw: &mut dyn B2drawTrait) {
		private::draw(self, draw);
	}
}
