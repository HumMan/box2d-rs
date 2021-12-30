use crate::joints::b2_mouse_joint::*;
use crate::b2_time_step::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_math::*;

// // p = attached point, m = mouse point
// // c = p - m
// // cdot = v
// //      = v + cross(w, r)
// // J = [i r_skew]
// // Identity used:
// // w k % (rx i + ry j) = w * (-ry i + rx j)

pub(crate) fn  init_velocity_constraints<D: UserDataType>(this: &mut B2mouseJoint<D>,data: &B2solverData, positions: &mut [B2position], velocities: &mut [B2velocity]) 
{
	let m_body_b = this.base.m_body_b.borrow();
	this.m_index_b = m_body_b.m_island_index;
	this.m_local_center_b = m_body_b.m_sweep.local_center;
	this.m_inv_mass_b = m_body_b.m_inv_mass;
	this.m_inv_ib = m_body_b.m_inv_i;

	let c_b: B2vec2 =positions[this.m_index_b as usize].c;
	let a_b: f32 =positions[this.m_index_b as usize].a;
	let mut v_b: B2vec2 =velocities[this.m_index_b as usize].v;
	let mut w_b: f32 =velocities[this.m_index_b as usize].w;

	let q_b = B2Rot::new(a_b);

	//let mass: f32 =m_body_b.get_mass();

	let d:f32 = this.m_damping;
	let k:f32 = this.m_stiffness;

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	let h: f32 =data.step.dt;
	this.m_gamma = h * (d + h * k);
	if this.m_gamma != 0.0
	{
		this.m_gamma = 1.0 / this.m_gamma;
	}
	this.m_beta = h * k * this.m_gamma;

	// Compute the effective mass matrix.
	this.m_r_b = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);

	// k    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	let mut k = B2Mat22::default();
	k.ex.x = this.m_inv_mass_b + this.m_inv_ib * this.m_r_b.y * this.m_r_b.y + this.m_gamma;
	k.ex.y = -this.m_inv_ib * this.m_r_b.x * this.m_r_b.y;
	k.ey.x = k.ex.y;
	k.ey.y = this.m_inv_mass_b + this.m_inv_ib * this.m_r_b.x * this.m_r_b.x + this.m_gamma;

	this.m_mass = k.get_inverse();

	this.m_c = c_b + this.m_r_b - this.m_target_a;
	this.m_c *= this.m_beta;

	// Cheat with some damping
	w_b *= 0.98;

	if data.step.warm_starting
	{
		this.m_impulse *= data.step.dt_ratio;
		v_b += this.m_inv_mass_b * this.m_impulse;
		w_b += this.m_inv_ib * b2_cross(this.m_r_b, this.m_impulse);
	}
	else
	{
		this.m_impulse.set_zero();
	}

	velocities[this.m_index_b as usize].v = v_b;
	velocities[this.m_index_b as usize].w = w_b;
}

pub(crate) fn  solve_velocity_constraints<D: UserDataType>(this: &mut B2mouseJoint<D>,data: &B2solverData, velocities: &mut [B2velocity]) 
{
	let mut v_b: B2vec2 =velocities[this.m_index_b as usize].v;
	let mut w_b: f32 =velocities[this.m_index_b as usize].w;

	// cdot = v + cross(w, r)
	let cdot: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, this.m_r_b);
	let mut impulse: B2vec2 =b2_mul(this.m_mass, -(cdot + this.m_c + this.m_gamma * this.m_impulse));

	let old_impulse: B2vec2 =this.m_impulse;
	this.m_impulse += impulse;
	let max_impulse: f32 =data.step.dt * this.m_max_force;
	if this.m_impulse.length_squared() > max_impulse * max_impulse
	{
		this.m_impulse *= max_impulse / this.m_impulse.length();
	}
	impulse =this. m_impulse - old_impulse;

	v_b += this.m_inv_mass_b * impulse;
	w_b += this.m_inv_ib * b2_cross(this.m_r_b, impulse);

	velocities[this.m_index_b as usize].v = v_b;
	velocities[this.m_index_b as usize].w = w_b;
}

pub(crate) fn  solve_position_constraints<D: UserDataType>(_this: &mut B2mouseJoint<D>, data: &B2solverData, _positions: &mut [B2position]) -> bool
{
	b2_not_used(data);
	return true;
}
