use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_motor_joint::*;

// Point-to-point constraint
// cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-i -r1_skew i r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// r1 = offset - c1
// r2 = -c2

// Angle constraint
// cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// k = invI1 + invI2

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	self_: &mut B2motorJoint<D>,
	data: &B2solverData,
	positions: &[B2position],
	velocities: &mut [B2velocity],
) {
	{
		let m_body_a = self_.base.m_body_a.borrow();
		let m_body_b = self_.base.m_body_b.borrow();
		self_.m_index_a = m_body_a.m_island_index;
		self_.m_index_b = m_body_b.m_island_index;
		self_.m_local_center_a = m_body_a.m_sweep.local_center;
		self_.m_local_center_b = m_body_b.m_sweep.local_center;
		self_.m_inv_mass_a = m_body_a.m_inv_mass;
		self_.m_inv_mass_b = m_body_b.m_inv_mass;
		self_.m_inv_ia = m_body_a.m_inv_i;
		self_.m_inv_ib = m_body_b.m_inv_i;
	}

	let c_a: B2vec2 = positions[self_.m_index_a as usize].c;
	let a_a: f32 = positions[self_.m_index_a as usize].a;
	let mut v_a: B2vec2 = velocities[self_.m_index_a as usize].v;
	let mut w_a: f32 = velocities[self_.m_index_a as usize].w;

	let c_b: B2vec2 = positions[self_.m_index_b as usize].c;
	let a_b: f32 = positions[self_.m_index_b as usize].a;
	let mut v_b: B2vec2 = velocities[self_.m_index_b as usize].v;
	let mut w_b: f32 = velocities[self_.m_index_b as usize].w;

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	// Compute the effective mass matrix.
	self_.m_r_a = b2_mul_rot_by_vec2(q_a, self_.m_linear_offset - self_.m_local_center_a);
	self_.m_r_b = b2_mul_rot_by_vec2(q_b, -self_.m_local_center_b);

	// J = [-i -r1_skew i r2_skew]
	// r_skew = [-ry; rx]

	// Matlab
	// k = [ m_a+r1y^2*i_a+m_b+r2y^2*i_b,  -r1y*i_a*r1x-r2y*i_b*r2x,          -r1y*i_a-r2y*i_b]
	//     [  -r1y*i_a*r1x-r2y*i_b*r2x, m_a+r1x^2*i_a+m_b+r2x^2*i_b,           r1x*i_a+r2x*i_b]
	//     [          -r1y*i_a-r2y*i_b,           r1x*i_a+r2x*i_b,                   i_a+i_b]

	let m_a: f32 = self_.m_inv_mass_a;
	let m_b: f32 = self_.m_inv_mass_b;
	let i_a: f32 = self_.m_inv_ia;
	let i_b: f32 = self_.m_inv_ib;

	// Upper 2 by 2 of k for point to point
	let mut k = B2Mat22::default();
	k.ex.x = m_a + m_b + i_a * self_.m_r_a.y * self_.m_r_a.y + i_b * self_.m_r_b.y * self_.m_r_b.y;
	k.ex.y = -i_a * self_.m_r_a.x * self_.m_r_a.y - i_b * self_.m_r_b.x * self_.m_r_b.y;
	k.ey.x = k.ex.y;
	k.ey.y = m_a + m_b + i_a * self_.m_r_a.x * self_.m_r_a.x + i_b * self_.m_r_b.x * self_.m_r_b.x;

	self_.m_linear_mass = k.get_inverse();

	self_.m_angular_mass = i_a + i_b;
	if self_.m_angular_mass > 0.0 {
		self_.m_angular_mass = 1.0 / self_.m_angular_mass;
	}

	self_.m_linear_error = c_b + self_.m_r_b - c_a - self_.m_r_a;
	self_.m_angular_error = a_b - a_a - self_.m_angular_offset;

	if data.step.warm_starting {
		// Scale impulses to support a variable time step.
		self_.m_linear_impulse *= data.step.dt_ratio;
		self_.m_angular_impulse *= data.step.dt_ratio;

		let p = B2vec2::new(self_.m_linear_impulse.x, self_.m_linear_impulse.y);
		v_a -= m_a * p;
		w_a -= i_a * (b2_cross(self_.m_r_a, p) + self_.m_angular_impulse);
		v_b += m_b * p;
		w_b += i_b * (b2_cross(self_.m_r_b, p) + self_.m_angular_impulse);
	} else {
		self_.m_linear_impulse.set_zero();
		self_.m_angular_impulse = 0.0;
	}

	velocities[self_.m_index_a as usize].v = v_a;
	velocities[self_.m_index_a as usize].w = w_a;
	velocities[self_.m_index_b as usize].v = v_b;
	velocities[self_.m_index_b as usize].w = w_b;
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	self_: &mut B2motorJoint<D>,
	data: &B2solverData,
	velocities: &mut [B2velocity],
) {
	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[self_.m_index_a as usize];
	// let mut v_a: B2vec2 =velocities[self_.m_index_a as usize].v;
	// let mut w_a: f32 =velocities[self_.m_index_a as usize].w;

	//TODO_humman - optimize others like this
	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[self_.m_index_b as usize];
	// let mut v_b: B2vec2 =velocities[self_.m_index_b as usize].v;
	// let mut w_b: f32 =velocities[self_.m_index_b as usize].w;

	//TODO_humman - optimize others like this
	let B2motorJoint {
		m_inv_mass_a: m_a,
		m_inv_mass_b: m_b,
		m_inv_ia: i_a,
		m_inv_ib: i_b,
		..
	} = *self_;
	// let m_a:f32 = self_.m_inv_mass_a;
	// let m_b: f32 =self_.m_inv_mass_b;
	// let i_a: f32 = self_.m_inv_ia;
	// let i_b: f32 =self_.m_inv_ib;

	let h: f32 = data.step.dt;
	let inv_h: f32 = data.step.inv_dt;

	// solve angular friction
	{
		let cdot: f32 = w_b - w_a + inv_h * self_.m_correction_factor * self_.m_angular_error;
		let mut impulse: f32 = -self_.m_angular_mass * cdot;

		let old_impulse: f32 = self_.m_angular_impulse;
		let max_impulse: f32 = h * self_.m_max_torque;
		self_.m_angular_impulse =
			b2_clamp(self_.m_angular_impulse + impulse, -max_impulse, max_impulse);
		impulse = self_.m_angular_impulse - old_impulse;

		w_a -= i_a * impulse;
		w_b += i_b * impulse;
	}

	// solve linear friction
	{
		let cdot: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b)
			- v_a - b2_cross_scalar_by_vec(w_a, self_.m_r_a)
			+ inv_h * self_.m_correction_factor * self_.m_linear_error;

		let mut impulse: B2vec2 = -b2_mul(self_.m_linear_mass, cdot);
		let old_impulse: B2vec2 = self_.m_linear_impulse;
		self_.m_linear_impulse += impulse;

		let max_impulse: f32 = h * self_.m_max_force;

		if self_.m_linear_impulse.length_squared() > max_impulse * max_impulse {
			self_.m_linear_impulse.normalize();
			self_.m_linear_impulse *= max_impulse;
		}

		impulse = self_.m_linear_impulse - old_impulse;

		v_a -= m_a * impulse;
		w_a -= i_a * b2_cross(self_.m_r_a, impulse);

		v_b += m_b * impulse;
		w_b += i_b * b2_cross(self_.m_r_b, impulse);
	}

	velocities[self_.m_index_a as usize].v = v_a;
	velocities[self_.m_index_a as usize].w = w_a;
	velocities[self_.m_index_b as usize].v = v_b;
	velocities[self_.m_index_b as usize].w = w_b;
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	_self: &B2motorJoint<D>,
	data: &B2solverData,
	_positions: &mut [B2position],
) -> bool {
	b2_not_used(data);

	return true;
}