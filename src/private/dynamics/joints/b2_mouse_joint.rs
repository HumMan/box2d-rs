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

pub(crate) fn  init_velocity_constraints<D: UserDataType>(self_: &mut B2mouseJoint<D>,data: &B2solverData, positions: &[B2position], velocities: &mut [B2velocity]) 
{
	let m_body_b = self_.base.m_body_b.borrow();
	self_.m_index_b = m_body_b.m_island_index;
	self_.m_local_center_b = m_body_b.m_sweep.local_center;
	self_.m_inv_mass_b = m_body_b.m_inv_mass;
	self_.m_inv_ib = m_body_b.m_inv_i;

	let c_b: B2vec2 =positions[self_.m_index_b as usize].c;
	let a_b: f32 =positions[self_.m_index_b as usize].a;
	let mut v_b: B2vec2 =velocities[self_.m_index_b as usize].v;
	let mut w_b: f32 =velocities[self_.m_index_b as usize].w;

	let q_b = B2Rot::new(a_b);

	//let mass: f32 =m_body_b.get_mass();

	let d:f32 = self_.m_damping;
	let k:f32 = self_.m_stiffness;

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	let h: f32 =data.step.dt;
	self_.m_gamma = h * (d + h * k);
	if self_.m_gamma != 0.0
	{
		self_.m_gamma = 1.0 / self_.m_gamma;
	}
	self_.m_beta = h * k * self_.m_gamma;

	// Compute the effective mass matrix.
	self_.m_r_b = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);

	// k    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	let mut k = B2Mat22::default();
	k.ex.x = self_.m_inv_mass_b + self_.m_inv_ib * self_.m_r_b.y * self_.m_r_b.y + self_.m_gamma;
	k.ex.y = -self_.m_inv_ib * self_.m_r_b.x * self_.m_r_b.y;
	k.ey.x = k.ex.y;
	k.ey.y = self_.m_inv_mass_b + self_.m_inv_ib * self_.m_r_b.x * self_.m_r_b.x + self_.m_gamma;

	self_.m_mass = k.get_inverse();

	self_.m_c = c_b + self_.m_r_b - self_.m_target_a;
	self_.m_c *= self_.m_beta;

	// Cheat with some damping
	w_b *= 0.98;

	if data.step.warm_starting
	{
		self_.m_impulse *= data.step.dt_ratio;
		v_b += self_.m_inv_mass_b * self_.m_impulse;
		w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, self_.m_impulse);
	}
	else
	{
		self_.m_impulse.set_zero();
	}

	velocities[self_.m_index_b as usize].v = v_b;
	velocities[self_.m_index_b as usize].w = w_b;
}

pub(crate) fn  solve_velocity_constraints<D: UserDataType>(self_: &mut B2mouseJoint<D>,data: &B2solverData, velocities: &mut [B2velocity]) 
{
	let mut v_b: B2vec2 =velocities[self_.m_index_b as usize].v;
	let mut w_b: f32 =velocities[self_.m_index_b as usize].w;

	// cdot = v + cross(w, r)
	let cdot: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b);
	let mut impulse: B2vec2 =b2_mul(self_.m_mass, -(cdot + self_.m_c + self_.m_gamma * self_.m_impulse));

	let old_impulse: B2vec2 =self_.m_impulse;
	self_.m_impulse += impulse;
	let max_impulse: f32 =data.step.dt * self_.m_max_force;
	if self_.m_impulse.length_squared() > max_impulse * max_impulse
	{
		self_.m_impulse *= max_impulse / self_.m_impulse.length();
	}
	impulse =self_. m_impulse - old_impulse;

	v_b += self_.m_inv_mass_b * impulse;
	w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, impulse);

	velocities[self_.m_index_b as usize].v = v_b;
	velocities[self_.m_index_b as usize].w = w_b;
}

pub(crate) fn  solve_position_constraints<D: UserDataType>(_self: &mut B2mouseJoint<D>, data: &B2solverData, _positions: &mut [B2position]) -> bool
{
	b2_not_used(data);
	return true;
}
