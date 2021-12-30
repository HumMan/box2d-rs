
use crate::joints::b2_gear_joint::*;
use crate::b2_time_step::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_math::*;
use crate::b2_joint::*;
use crate::b2_body::B2bodyType;


// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// c = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [j1 ratio * j2]
// k = J * inv_m * JT
//   = j1 * invM1 * J1T + ratio * ratio * j2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// cdot = angular_velocity
// J = [0 0 1]
// k = J * inv_m * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// k = J * inv_m * JT = inv_mass + invI * cross(r, ug)^2

pub(crate) fn new<D: UserDataType>(def: &B2gearJointDef<D>)->B2gearJoint<D>
{
	let m_joint1 = def.joint1.as_ref().unwrap().clone();
	let m_joint2 = def.joint2.as_ref().unwrap().clone();

	let m_type_a = m_joint1.borrow().get_base().get_type();
	let m_type_b = m_joint2.borrow().get_base().get_type();

	b2_assert(m_type_a == B2jointType::ERevoluteJoint || m_type_a == B2jointType::EPrismaticJoint);
	b2_assert(m_type_b == B2jointType::ERevoluteJoint || m_type_b == B2jointType::EPrismaticJoint);

	let coordinate_a: f32;
	let coordinate_b: f32;

	// TODO_ERIN there might be some problem with the joint edges in B2joint.

	let m_body_c = m_joint1.borrow().get_base().get_body_a();
	let m_body_a = m_joint1.borrow().get_base().get_body_b();

	// Body B on joint1 must be dynamic
	b2_assert(m_body_a.borrow().m_type == B2bodyType::B2DynamicBody);

	// Get geometry of joint1
	let xf_a: B2Transform = m_body_a.borrow().m_xf;
	let a_a: f32 =m_body_a.borrow().m_sweep.a;
	let xf_c:B2Transform = m_body_c.borrow().m_xf;
	let a_c: f32 =m_body_c.borrow().m_sweep.a;

	let m_local_anchor_c;
	let m_local_anchor_a;
	let m_reference_angle_a;
	let m_local_axis_c;

	match  m_joint1.borrow().as_derived()
	{
		JointAsDerived::ERevoluteJoint(ref revolute) => {
				m_local_anchor_c = revolute.m_local_anchor_a;
				m_local_anchor_a = revolute.m_local_anchor_b;
				m_reference_angle_a = revolute.m_reference_angle;
				m_local_axis_c = B2vec2::zero();

				coordinate_a = a_a - a_c - m_reference_angle_a;
		}
		JointAsDerived::EPrismaticJoint(ref prismatic) => {
				m_local_anchor_c = prismatic.m_local_anchor_a;
				m_local_anchor_a = prismatic.m_local_anchor_b;
				m_reference_angle_a = prismatic.m_reference_angle;
				m_local_axis_c = prismatic.m_local_xaxis_a;

				let p_c: B2vec2 =m_local_anchor_c;
				let p_a: B2vec2 =b2_mul_t_rot_by_vec2(xf_c.q, b2_mul_rot_by_vec2(xf_a.q, m_local_anchor_a) + (xf_a.p - xf_c.p));
				coordinate_a = b2_dot(p_a - p_c, m_local_axis_c);
		}
		_=> panic!()
	}

	let m_body_d = m_joint2.borrow().get_base().get_body_a();
	let m_body_b = m_joint2.borrow().get_base().get_body_b();

	// Body B on joint2 must be dynamic
	b2_assert(m_body_b.borrow().m_type == B2bodyType::B2DynamicBody);

	// Get geometry of joint2
	let xf_b: B2Transform = m_body_b.borrow().m_xf;
	let a_b: f32 =m_body_b.borrow().m_sweep.a;
	let xf_d: B2Transform = m_body_d.borrow().m_xf;
	let a_d: f32 =m_body_d.borrow().m_sweep.a;

	let m_local_anchor_d;
	let m_local_anchor_b;
	let m_reference_angle_b;
	let m_local_axis_d;

	match  m_joint2.borrow().as_derived()
	{
		JointAsDerived::ERevoluteJoint(ref revolute) => {
			m_local_anchor_d = revolute.m_local_anchor_a;
			m_local_anchor_b = revolute.m_local_anchor_b;
			m_reference_angle_b = revolute.m_reference_angle;
			m_local_axis_d = B2vec2::zero();
	
			coordinate_b = a_b - a_d - m_reference_angle_b;
		}
		JointAsDerived::EPrismaticJoint(ref prismatic) => {
			m_local_anchor_d = prismatic.m_local_anchor_a;
			m_local_anchor_b = prismatic.m_local_anchor_b;
			m_reference_angle_b = prismatic.m_reference_angle;
			m_local_axis_d = prismatic.m_local_xaxis_a;
	
			let p_d: B2vec2 =m_local_anchor_d;
			let p_b: B2vec2 =b2_mul_t_rot_by_vec2(xf_d.q, b2_mul_rot_by_vec2(xf_b.q, m_local_anchor_b) + (xf_b.p - xf_d.p));
			coordinate_b = b2_dot(p_b - p_d, m_local_axis_d);
		}
		_=> panic!()
	}

	let m_ratio = def.ratio;

	let m_constant = coordinate_a + m_ratio * coordinate_b;

	let m_impulse = 0.0;

	return B2gearJoint{

		base: B2joint::new(&def.base),

		m_joint1,
		m_joint2,
	
		m_type_a,
		m_type_b,

		m_body_c,
		m_body_d,
	
		m_local_anchor_a,
		m_local_anchor_b,
		m_local_anchor_c,
		m_local_anchor_d,
	
		m_local_axis_c,
		m_local_axis_d,
	
		m_reference_angle_a,
		m_reference_angle_b,
	
		m_constant,
		m_ratio,
	
		m_impulse,
	
		m_index_a: 0,
		m_index_b: 0,
		m_index_c: 0,
		m_index_d: 0,
		m_lc_a: B2vec2::zero(),
		m_lc_b: B2vec2::zero(),
		m_lc_c: B2vec2::zero(),
		m_lc_d: B2vec2::zero(),
		m_m_a: 0.0,
		m_m_b: 0.0,
		m_m_c: 0.0,
		m_m_d: 0.0,
		m_i_a: 0.0,
		m_i_b: 0.0,
		m_i_c: 0.0,
		m_i_d: 0.0,
		m_jv_ac: B2vec2::zero(),
		m_jv_bd: B2vec2::zero(),
		m_jw_a: 0.0,
		m_jw_b: 0.0,
		m_jw_c: 0.0,
		m_jw_d: 0.0,
		m_mass: 0.0,
	};
}

pub(crate) fn init_velocity_constraints<D: UserDataType>(self_: &mut B2gearJoint<D>, 
	data: &B2solverData, positions: &[B2position], velocities: &mut [B2velocity])
{
	let m_body_a = self_.base.m_body_a.borrow();
	let m_body_b = self_.base.m_body_b.borrow();
	let m_body_c = self_.m_body_c.borrow();
	let m_body_d = self_.m_body_d.borrow();
	self_.m_index_a = m_body_a.m_island_index;
	self_.m_index_b = m_body_b.m_island_index;
	self_.m_index_c = m_body_c.m_island_index;
	self_.m_index_d = m_body_d.m_island_index;
	self_.m_lc_a = m_body_a.m_sweep.local_center;
	self_.m_lc_b = m_body_b.m_sweep.local_center;
	self_.m_lc_c = m_body_c.m_sweep.local_center;
	self_.m_lc_d = m_body_d.m_sweep.local_center;
	self_.m_m_a = m_body_a.m_inv_mass;
	self_.m_m_b = m_body_b.m_inv_mass;
	self_.m_m_c = m_body_c.m_inv_mass;
	self_.m_m_d = m_body_d.m_inv_mass;
	self_.m_i_a = m_body_a.m_inv_i;
	self_.m_i_b = m_body_b.m_inv_i;
	self_.m_i_c = m_body_c.m_inv_i;
	self_.m_i_d = m_body_d.m_inv_i;

	let a_a: f32 =positions[self_.m_index_a as usize].a;
	let mut v_a: B2vec2 =velocities[self_.m_index_a as usize].v;
	let mut w_a: f32 =velocities[self_.m_index_a as usize].w;

	let a_b: f32 =positions[self_.m_index_b as usize].a;
	let mut v_b: B2vec2 =velocities[self_.m_index_b as usize].v;
	let mut w_b: f32 =velocities[self_.m_index_b as usize].w;

	let a_c: f32 =positions[self_.m_index_c as usize].a;
	let mut v_c: B2vec2 =velocities[self_.m_index_c as usize].v;
	let mut w_c: f32 =velocities[self_.m_index_c as usize].w;

	let a_d: f32 =positions[self_.m_index_d as usize].a;
	let mut v_d: B2vec2 =velocities[self_.m_index_d as usize].v;
	let mut w_d: f32 =velocities[self_.m_index_d as usize].w;

	let (q_a, q_b, q_c, q_d) = (B2Rot::new(a_a),B2Rot::new(a_b),B2Rot::new(a_c),B2Rot::new(a_d));

	self_.m_mass = 0.0;

	if self_.m_type_a == B2jointType::ERevoluteJoint
	{
		self_.m_jv_ac.set_zero();
		self_.m_jw_a = 1.0;
		self_.m_jw_c = 1.0;
		self_.m_mass += self_.m_i_a + self_.m_i_c;
	}
	else
	{
		let u: B2vec2 =b2_mul_rot_by_vec2(q_c, self_.m_local_axis_c);
		let r_c: B2vec2 =b2_mul_rot_by_vec2(q_c, self_.m_local_anchor_c - self_.m_lc_c);
		let r_a: B2vec2 =b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_lc_a);
		self_.m_jv_ac = u;
		self_.m_jw_c = b2_cross(r_c, u);
		self_.m_jw_a = b2_cross(r_a, u);
		self_.m_mass += self_.m_m_c + self_.m_m_a + self_.m_i_c * self_.m_jw_c * self_.m_jw_c + self_.m_i_a * self_.m_jw_a * self_.m_jw_a;
	}

	if self_.m_type_b == B2jointType::ERevoluteJoint
	{
		self_.m_jv_bd.set_zero();
		self_.m_jw_b = self_.m_ratio;
		self_.m_jw_d = self_.m_ratio;
		self_.m_mass += self_.m_ratio * self_.m_ratio * (self_.m_i_b + self_.m_i_d);
	}
	else
	{
		let u: B2vec2 =b2_mul_rot_by_vec2(q_d, self_.m_local_axis_d);
		let r_d: B2vec2 =b2_mul_rot_by_vec2(q_d, self_.m_local_anchor_d - self_.m_lc_d);
		let r_b: B2vec2 =b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_lc_b);
		self_.m_jv_bd = self_.m_ratio * u;
		self_.m_jw_d = self_.m_ratio * b2_cross(r_d, u);
		self_.m_jw_b = self_.m_ratio * b2_cross(r_b, u);
		self_.m_mass += self_.m_ratio * self_.m_ratio * (self_.m_m_d + self_.m_m_b) + 
		self_.m_i_d * self_.m_jw_d * self_.m_jw_d + self_.m_i_b * self_.m_jw_b * self_.m_jw_b;
	}

	// Compute effective mass.
	self_.m_mass = if self_.m_mass > 0.0 { 1.0 / self_.m_mass } else {0.0};

	if data.step.warm_starting
	{
		v_a += (self_.m_m_a * self_.m_impulse) * self_.m_jv_ac;
		w_a += self_.m_i_a * self_.m_impulse * self_.m_jw_a;
		v_b += (self_.m_m_b * self_.m_impulse) * self_.m_jv_bd;
		w_b += self_.m_i_b * self_.m_impulse * self_.m_jw_b;
		v_c -= (self_.m_m_c * self_.m_impulse) * self_.m_jv_ac;
		w_c -= self_.m_i_c * self_.m_impulse * self_.m_jw_c;
		v_d -= (self_.m_m_d * self_.m_impulse) * self_.m_jv_bd;
		w_d -= self_.m_i_d * self_.m_impulse * self_.m_jw_d;
	}
	else
	{
		self_.m_impulse = 0.0;
	}

	velocities[self_.m_index_a as usize].v = v_a;
	velocities[self_.m_index_a as usize].w = w_a;
	velocities[self_.m_index_b as usize].v = v_b;
	velocities[self_.m_index_b as usize].w = w_b;
	velocities[self_.m_index_c as usize].v = v_c;
	velocities[self_.m_index_c as usize].w = w_c;
	velocities[self_.m_index_d as usize].v = v_d;
	velocities[self_.m_index_d as usize].w = w_d;
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(self_: &mut B2gearJoint<D>, 
	_data: &B2solverData, velocities: &mut [B2velocity])
{
	let mut v_a: B2vec2 =velocities[self_.m_index_a as usize].v;
	let mut w_a: f32 =velocities[self_.m_index_a as usize].w;
	let mut v_b: B2vec2 =velocities[self_.m_index_b as usize].v;
	let mut w_b: f32 =velocities[self_.m_index_b as usize].w;
	let mut v_c: B2vec2 =velocities[self_.m_index_c as usize].v;
	let mut w_c: f32 =velocities[self_.m_index_c as usize].w;
	let mut v_d: B2vec2 =velocities[self_.m_index_d as usize].v;
	let mut w_d: f32 =velocities[self_.m_index_d as usize].w;

	let mut cdot: f32 =b2_dot(self_.m_jv_ac, v_a - v_c) + b2_dot(self_.m_jv_bd, v_b - v_d);
	cdot += (self_.m_jw_a * w_a - self_.m_jw_c * w_c) + (self_.m_jw_b * w_b - self_.m_jw_d * w_d);

	let impulse: f32 =-self_.m_mass * cdot;
	self_.m_impulse += impulse;

	v_a += (self_.m_m_a * impulse) * self_.m_jv_ac;
	w_a += self_.m_i_a * impulse * self_.m_jw_a;
	v_b += (self_.m_m_b * impulse) * self_.m_jv_bd;
	w_b += self_.m_i_b * impulse * self_.m_jw_b;
	v_c -= (self_.m_m_c * impulse) * self_.m_jv_ac;
	w_c -= self_.m_i_c * impulse * self_.m_jw_c;
	v_d -= (self_.m_m_d * impulse) * self_.m_jv_bd;
	w_d -= self_.m_i_d * impulse * self_.m_jw_d;

	velocities[self_.m_index_a as usize].v = v_a;
	velocities[self_.m_index_a as usize].w = w_a;
	velocities[self_.m_index_b as usize].v = v_b;
	velocities[self_.m_index_b as usize].w = w_b;
	velocities[self_.m_index_c as usize].v = v_c;
	velocities[self_.m_index_c as usize].w = w_c;
	velocities[self_.m_index_d as usize].v = v_d;
	velocities[self_.m_index_d as usize].w = w_d;
}

pub(crate) fn solve_position_constraints<D: UserDataType>(self_: &B2gearJoint<D>,
	_data: &B2solverData, positions: &mut [B2position])->bool
{
	let mut c_a: B2vec2 =positions[self_.m_index_a as usize].c;
	let mut a_a: f32 =positions[self_.m_index_a as usize].a;
	let mut c_b: B2vec2 =positions[self_.m_index_b as usize].c;
	let mut a_b: f32 =positions[self_.m_index_b as usize].a;
	let mut c_c: B2vec2 =positions[self_.m_index_c as usize].c;
	let mut a_c: f32 =positions[self_.m_index_c as usize].a;
	let mut c_d: B2vec2 =positions[self_.m_index_d as usize].c;
	let mut a_d: f32 =positions[self_.m_index_d as usize].a;

	let (q_a, q_b, q_c, q_d) = (B2Rot::new(a_a),B2Rot::new(a_b),B2Rot::new(a_c),B2Rot::new(a_d));

	let linear_error: f32 =0.0;

	let coordinate_a: f32;
	let coordinate_b: f32;

	let jv_ac: B2vec2;
	let jv_bd: B2vec2;
	let jw_a: f32;
	let jw_b: f32;
	let jw_c: f32;
	let jw_d: f32;
	let mut mass: f32 =0.0;

	if self_.m_type_a == B2jointType::ERevoluteJoint
	{
		jv_ac = B2vec2::zero();
		jw_a = 1.0;
		jw_c = 1.0;
		mass += self_.m_i_a + self_.m_i_c;

		coordinate_a = a_a - a_c - self_.m_reference_angle_a;
	}
	else
	{
		let u: B2vec2 =b2_mul_rot_by_vec2(q_c, self_.m_local_axis_c);
		let r_c: B2vec2 =b2_mul_rot_by_vec2(q_c, self_.m_local_anchor_c - self_.m_lc_c);
		let r_a: B2vec2 =b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_lc_a);
		jv_ac = u;
		jw_c = b2_cross(r_c, u);
		jw_a = b2_cross(r_a, u);
		mass += self_.m_m_c + self_.m_m_a + self_.m_i_c * jw_c * jw_c + self_.m_i_a * jw_a * jw_a;

		let p_c: B2vec2 =self_.m_local_anchor_c - self_.m_lc_c;
		let p_a: B2vec2 =b2_mul_t_rot_by_vec2(q_c, r_a + (c_a - c_c));
		coordinate_a = b2_dot(p_a - p_c, self_.m_local_axis_c);
	}

	if self_.m_type_b == B2jointType::ERevoluteJoint
	{
		jv_bd = B2vec2::zero();
		jw_b = self_.m_ratio;
		jw_d = self_.m_ratio;
		mass += self_.m_ratio * self_.m_ratio * (self_.m_i_b + self_.m_i_d);

		coordinate_b = a_b - a_d - self_.m_reference_angle_b;
	}
	else
	{
		let u: B2vec2 =b2_mul_rot_by_vec2(q_d, self_.m_local_axis_d);
		let r_d: B2vec2 =b2_mul_rot_by_vec2(q_d, self_.m_local_anchor_d - self_.m_lc_d);
		let r_b: B2vec2 =b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_lc_b);
		jv_bd = self_.m_ratio * u;
		jw_d = self_.m_ratio * b2_cross(r_d, u);
		jw_b = self_.m_ratio * b2_cross(r_b, u);
		mass += self_.m_ratio * self_.m_ratio * (self_.m_m_d + self_.m_m_b) + self_.m_i_d * jw_d * jw_d + self_.m_i_b * jw_b * jw_b;

		let p_d: B2vec2 =self_.m_local_anchor_d - self_.m_lc_d;
		let p_b: B2vec2 =b2_mul_t_rot_by_vec2(q_d, r_b + (c_b - c_d));
		coordinate_b = b2_dot(p_b - p_d, self_.m_local_axis_d);
	}

	let c: f32 =(coordinate_a + self_.m_ratio * coordinate_b) - self_.m_constant;

	let mut impulse: f32 =0.0;
	if mass > 0.0
	{
		impulse = -c / mass;
	}

	c_a += self_.m_m_a * impulse * jv_ac;
	a_a += self_.m_i_a * impulse * jw_a;
	c_b += self_.m_m_b * impulse * jv_bd;
	a_b += self_.m_i_b * impulse * jw_b;
	c_c -= self_.m_m_c * impulse * jv_ac;
	a_c -= self_.m_i_c * impulse * jw_c;
	c_d -= self_.m_m_d * impulse * jv_bd;
	a_d -= self_.m_i_d * impulse * jw_d;

	positions[self_.m_index_a as usize].c = c_a;
	positions[self_.m_index_a as usize].a = a_a;
	positions[self_.m_index_b as usize].c = c_b;
	positions[self_.m_index_b as usize].a = a_b;
	positions[self_.m_index_c as usize].c = c_c;
	positions[self_.m_index_c as usize].a = a_c;
	positions[self_.m_index_d as usize].c = c_d;
	positions[self_.m_index_d as usize].a = a_d;

	// TODO_ERIN not implemented
	return linear_error < B2_LINEAR_SLOP;
}