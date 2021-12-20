use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_friction_joint::*;

// Point-to-point constraint
// cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-i -r1_skew i r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// k = invI1 + invI2

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	this: &mut B2frictionJoint<D>,
	data: &mut B2solverData,
	positions: &mut [B2position],
	velocities: &mut [B2velocity],
) {
	this.m_index_a = this.base.m_body_a.borrow().m_island_index;
	this.m_index_b = this.base.m_body_b.borrow().m_island_index;
	this.m_local_center_a = this.base.m_body_a.borrow().m_sweep.local_center;
	this.m_local_center_b = this.base.m_body_b.borrow().m_sweep.local_center;
	this.m_inv_mass_a = this.base.m_body_a.borrow().m_inv_mass;
	this.m_inv_mass_b = this.base.m_body_b.borrow().m_inv_mass;
	this.m_inv_ia = this.base.m_body_a.borrow().m_inv_i;
	this.m_inv_ib = this.base.m_body_b.borrow().m_inv_i;

	let a_a: f32 = positions[this.m_index_a as usize].a;
	let mut v_a: B2vec2 = velocities[this.m_index_a as usize].v;
	let mut w_a: f32 = velocities[this.m_index_a as usize].w;

	let a_b: f32 = positions[this.m_index_b as usize].a;
	let mut v_b: B2vec2 = velocities[this.m_index_b as usize].v;
	let mut w_b: f32 = velocities[this.m_index_b as usize].w;

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	// Compute the effective mass matrix.
	this.m_r_a = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	this.m_r_b = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);

	// J = [-i -r1_skew i r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// k = [ m_a+r1y^2*i_a+m_b+r2y^2*i_b,  -r1y*i_a*r1x-r2y*i_b*r2x,          -r1y*i_a-r2y*i_b]
	//     [  -r1y*i_a*r1x-r2y*i_b*r2x, m_a+r1x^2*i_a+m_b+r2x^2*i_b,           r1x*i_a+r2x*i_b]
	//     [          -r1y*i_a-r2y*i_b,           r1x*i_a+r2x*i_b,                   i_a+i_b]

	let m_a: f32 = this.m_inv_mass_a;
	let m_b: f32 = this.m_inv_mass_b;
	let i_a: f32 = this.m_inv_ia;
	let i_b: f32 = this.m_inv_ib;

	let mut k = B2Mat22::default();
	k.ex.x = m_a + m_b + i_a * this.m_r_a.y * this.m_r_a.y + i_b * this.m_r_b.y * this.m_r_b.y;
	k.ex.y = -i_a * this.m_r_a.x * this.m_r_a.y - i_b * this.m_r_b.x * this.m_r_b.y;
	k.ey.x = k.ex.y;
	k.ey.y = m_a + m_b + i_a * this.m_r_a.x * this.m_r_a.x + i_b * this.m_r_b.x * this.m_r_b.x;

	this.m_linear_mass = k.get_inverse();

	this.m_angular_mass = i_a + i_b;
	if this.m_angular_mass > 0.0 {
		this.m_angular_mass = 1.0 / this.m_angular_mass;
	}

	if data.step.warm_starting {
		// Scale impulses to support a variable time step.
		this.m_linear_impulse *= data.step.dt_ratio;
		this.m_angular_impulse *= data.step.dt_ratio;

		let p = B2vec2::new(this.m_linear_impulse.x, this.m_linear_impulse.y);
		v_a -= m_a * p;
		w_a -= i_a * (b2_cross(this.m_r_a, p) + this.m_angular_impulse);
		v_b += m_b * p;
		w_b += i_b * (b2_cross(this.m_r_b, p) + this.m_angular_impulse);
	} else {
		this.m_linear_impulse.set_zero();
		this.m_angular_impulse = 0.0;
	}

	velocities[this.m_index_a as usize].v = v_a;
	velocities[this.m_index_a as usize].w = w_a;
	velocities[this.m_index_b as usize].v = v_b;
	velocities[this.m_index_b as usize].w = w_b;
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	this: &mut B2frictionJoint<D>,
	data: &mut B2solverData,
	velocities: &mut [B2velocity],
) {
	let mut v_a: B2vec2 = velocities[this.m_index_a as usize].v;
	let mut w_a: f32 = velocities[this.m_index_a as usize].w;
	let mut v_b: B2vec2 = velocities[this.m_index_b as usize].v;
	let mut w_b: f32 = velocities[this.m_index_b as usize].w;

	let m_a: f32 = this.m_inv_mass_a;
	let m_b: f32 = this.m_inv_mass_b;
	let i_a: f32 = this.m_inv_ia;
	let i_b: f32 = this.m_inv_ib;

	let h: f32 = data.step.dt;

	// solve angular friction
	{
		let cdot: f32 = w_b - w_a;
		let mut impulse: f32 = -this.m_angular_mass * cdot;

		let old_impulse: f32 = this.m_angular_impulse;
		let max_impulse: f32 = h * this.m_max_torque;
		this.m_angular_impulse =
			b2_clamp(this.m_angular_impulse + impulse, -max_impulse, max_impulse);
		impulse = this.m_angular_impulse - old_impulse;

		w_a -= i_a * impulse;
		w_b += i_b * impulse;
	}

	// solve linear friction
	{
		let cdot: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, this.m_r_b)
			- v_a - b2_cross_scalar_by_vec(w_a, this.m_r_a);

		let mut impulse: B2vec2 = -b2_mul(this.m_linear_mass, cdot);
		let old_impulse: B2vec2 = this.m_linear_impulse;
		this.m_linear_impulse += impulse;

		let max_impulse: f32 = h * this.m_max_force;

		if this.m_linear_impulse.length_squared() > max_impulse * max_impulse {
			this.m_linear_impulse.normalize();
			this.m_linear_impulse *= max_impulse;
		}

		impulse = this.m_linear_impulse - old_impulse;

		v_a -= m_a * impulse;
		w_a -= i_a * b2_cross(this.m_r_a, impulse);

		v_b += m_b * impulse;
		w_b += i_b * b2_cross(this.m_r_b, impulse);
	}

	velocities[this.m_index_a as usize].v = v_a;
	velocities[this.m_index_a as usize].w = w_a;
	velocities[this.m_index_b as usize].v = v_b;
	velocities[this.m_index_b as usize].w = w_b;
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	_this: &B2frictionJoint<D>,
	data: &mut B2solverData,
	_positions: &mut [B2position],
) -> bool {
	b2_not_used(data);

	return true;
}
