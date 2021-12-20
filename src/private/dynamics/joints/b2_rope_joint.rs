use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_rope_joint::*;

// Limit:
// c = norm(p_b - p_a) - l
// u = (p_b - p_a) / norm(p_b - p_a)
// cdot = dot(u, v_b + cross(w_b, r_b) - v_a - cross(w_a, r_a))
// J = [-u -cross(r_a, u) u cross(r_b, u)]
// k = J * inv_m * JT
//   = inv_mass_a + inv_ia * cross(r_a, u)^2 + inv_mass_b + inv_ib * cross(r_b, u)^2

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	this: &mut B2ropeJoint<D>,
	data: &mut B2solverData,
	positions: &mut [B2position],
	velocities: &mut [B2velocity],
) {
	let m_body_a = this.base.m_body_a.borrow();
	let m_body_b = this.base.m_body_b.borrow();
	this.m_index_a = m_body_a.m_island_index;
	this.m_index_b = m_body_b.m_island_index;
	this.m_local_center_a = m_body_a.m_sweep.local_center;
	this.m_local_center_b = m_body_b.m_sweep.local_center;
	this.m_inv_mass_a = m_body_a.m_inv_mass;
	this.m_inv_mass_b = m_body_b.m_inv_mass;
	this.m_inv_ia = m_body_a.m_inv_i;
	this.m_inv_ib = m_body_b.m_inv_i;

	let B2position { c: c_a, a: a_a } = positions[this.m_index_a as usize];
	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[this.m_index_a as usize];

	let B2position { c: c_b, a: a_b } = positions[this.m_index_b as usize];
	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[this.m_index_b as usize];

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	this.m_r_a = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	this.m_r_b = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);

	this.m_u = c_b + this.m_r_b - c_a - this.m_r_a;

	this.m_length = this.m_u.length();

	//let c: f32 = this.m_length - this.m_max_length;

	if this.m_length > B2_LINEAR_SLOP {
		this.m_u *= 1.0 / this.m_length;
	} else {
		this.m_u.set_zero();
		this.m_mass = 0.0;
		this.m_impulse = 0.0;
		return;
	}

	// Compute effective mass.
	let cr_a: f32 = b2_cross(this.m_r_a, this.m_u);
	let cr_b: f32 = b2_cross(this.m_r_b, this.m_u);
	let inv_mass: f32 = this.m_inv_mass_a
		+ this.m_inv_ia * cr_a * cr_a
		+ this.m_inv_mass_b
		+ this.m_inv_ib * cr_b * cr_b;

	this.m_mass = if inv_mass != 0.0 { 1.0 / inv_mass } else { 0.0 };

	if data.step.warm_starting {
		// Scale the impulse to support a variable time step.
		this.m_impulse *= data.step.dt_ratio;

		let p: B2vec2 = this.m_impulse * this.m_u;
		v_a -= this.m_inv_mass_a * p;
		w_a -= this.m_inv_ia * b2_cross(this.m_r_a, p);
		v_b += this.m_inv_mass_b * p;
		w_b += this.m_inv_ib * b2_cross(this.m_r_b, p);
	} else {
		this.m_impulse = 0.0;
	}

	velocities[this.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[this.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	this: &mut B2ropeJoint<D>,
	data: &mut B2solverData,
	velocities: &mut [B2velocity],
) {
	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[this.m_index_a as usize];

	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[this.m_index_b as usize];

	// cdot = dot(u, v + cross(w, r))
	let vp_a: B2vec2 = v_a + b2_cross_scalar_by_vec(w_a, this.m_r_a);
	let vp_b: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, this.m_r_b);
	let c: f32 = this.m_length - this.m_max_length;
	let mut cdot: f32 = b2_dot(this.m_u, vp_b - vp_a);

	// Predictive constraint.
	if c < 0.0 {
		cdot += data.step.inv_dt * c;
	}

	let mut impulse: f32 = -this.m_mass * cdot;
	let old_impulse: f32 = this.m_impulse;
	this.m_impulse = b2_min(0.0, this.m_impulse + impulse);
	impulse = this.m_impulse - old_impulse;

	let p: B2vec2 = impulse * this.m_u;
	v_a -= this.m_inv_mass_a * p;
	w_a -= this.m_inv_ia * b2_cross(this.m_r_a, p);
	v_b += this.m_inv_mass_b * p;
	w_b += this.m_inv_ib * b2_cross(this.m_r_b, p);

	velocities[this.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[this.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	this: &mut B2ropeJoint<D>,
	_data: &mut B2solverData,
	positions: &mut [B2position],
) -> bool {
	let B2position {
		c: mut c_a,
		a: mut a_a,
	} = positions[this.m_index_a as usize];
	let B2position {
		c: mut c_b,
		a: mut a_b,
	} = positions[this.m_index_b as usize];

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);
	let mut u: B2vec2 = c_b + r_b - c_a - r_a;

	this.m_length = u.normalize();
	let mut c: f32 = this.m_length - this.m_max_length;

	c = b2_clamp(c, 0.0, B2_MAX_LINEAR_CORRECTION);

	let impulse: f32 = -this.m_mass * c;
	let p: B2vec2 = impulse * u;

	c_a -= this.m_inv_mass_a * p;
	a_a -= this.m_inv_ia * b2_cross(r_a, p);
	c_b += this.m_inv_mass_b * p;
	a_b += this.m_inv_ib * b2_cross(r_b, p);

	positions[this.m_index_a as usize] = B2position { c: c_a, a: a_a };
	positions[this.m_index_b as usize] = B2position { c: c_b, a: a_b };

	return this.m_length - this.m_max_length < B2_LINEAR_SLOP;
}