use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_pulley_joint::*;

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// c = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// k = J * inv_m * JT
//   = inv_mass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (inv_mass2 + invI2 * cross(r2, u2)^2)

pub(crate) fn new<D: UserDataType>(def: &B2pulleyJointDef<D>) -> B2pulleyJoint<D>
{
		let m_ground_anchor_a = def.ground_anchor_a;
		let m_ground_anchor_b = def.ground_anchor_b;
		let m_local_anchor_a = def.local_anchor_a;
		let m_local_anchor_b = def.local_anchor_b;

		let m_length_a = def.length_a;
		let m_length_b = def.length_b;

		b2_assert(def.ratio != 0.0);
		let m_ratio = def.ratio;

		let m_constant = def.length_a + m_ratio * def.length_b;

		let m_impulse = 0.0;
		return B2pulleyJoint{
			base: B2joint::new(&def.base),
			m_ground_anchor_a,
			m_ground_anchor_b,
			m_length_a,
			m_length_b,
		
			m_local_anchor_a,
			m_local_anchor_b,
			m_constant,
			m_ratio,
			m_impulse,
		
			m_index_a: 0,
			m_index_b: 0,
			m_u_a: B2vec2::zero(),
			m_u_b: B2vec2::zero(),
			m_r_a: B2vec2::zero(),
			m_r_b: B2vec2::zero(),
			m_local_center_a: B2vec2::zero(),
			m_local_center_b: B2vec2::zero(),
			m_inv_mass_a: 0.0,
			m_inv_mass_b: 0.0,
			m_inv_ia: 0.0,
			m_inv_ib: 0.0,
			m_mass: 0.0,
		};
}

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	this: &mut B2pulleyJoint<D>,
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

	let B2position {
		c: c_a,
		a: a_a,
	} = positions[this.m_index_a as usize];
	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[this.m_index_a as usize];

	let B2position {
		c: c_b,
		a: a_b,
	} = positions[this.m_index_b as usize];
	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[this.m_index_b as usize];

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	this.m_r_a = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	this.m_r_b = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);

	// Get the pulley axes.
	this.m_u_a = c_a + this.m_r_a - this.m_ground_anchor_a;
	this.m_u_b = c_b + this.m_r_b - this.m_ground_anchor_b;

	let length_a: f32 = this.m_u_a.length();
	let length_b: f32 = this.m_u_b.length();

	if length_a > 10.0 * B2_LINEAR_SLOP
	{
		this.m_u_a *= 1.0 / length_a;
	}
	else
	{
		this.m_u_a.set_zero();
	}

	if length_b > 10.0 * B2_LINEAR_SLOP
	{
		this.m_u_b *= 1.0 / length_b;
	}
	else
	{
		this.m_u_b.set_zero();
	}

	// Compute effective mass.
	let ru_a: f32 =b2_cross(this.m_r_a, this.m_u_a);
	let ru_b: f32 =b2_cross(this.m_r_b, this.m_u_b);

	let m_a: f32 =this.m_inv_mass_a + this.m_inv_ia * ru_a * ru_a;
	let m_b: f32 =this.m_inv_mass_b + this.m_inv_ib * ru_b * ru_b;

	this.m_mass = m_a + this.m_ratio * this.m_ratio * m_b;

	if this.m_mass > 0.0
	{
		this.m_mass = 1.0 / this.m_mass;
	}

	if data.step.warm_starting
	{
		// Scale impulses to support variable time steps.
		this.m_impulse *= data.step.dt_ratio;

		// Warm starting.
		let pa: B2vec2 =-(this.m_impulse) * this.m_u_a;
		let pb: B2vec2 =(-this.m_ratio * this.m_impulse) * this.m_u_b;

		v_a += this.m_inv_mass_a * pa;
		w_a += this.m_inv_ia * b2_cross(this.m_r_a, pa);
		v_b += this.m_inv_mass_b * pb;
		w_b += this.m_inv_ib * b2_cross(this.m_r_b, pb);
	}
	else
	{
		this.m_impulse = 0.0;
	}

	velocities[this.m_index_a as usize] = B2velocity {v: v_a, w: w_a};
	velocities[this.m_index_b as usize] = B2velocity {v: v_b, w: w_b};
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	this: &mut B2pulleyJoint<D>,
	_data: &mut B2solverData,
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

	let vp_a: B2vec2 =v_a + b2_cross_scalar_by_vec(w_a, this.m_r_a);
	let vp_b: B2vec2 =v_b + b2_cross_scalar_by_vec(w_b, this.m_r_b);

	let cdot: f32 =-b2_dot(this.m_u_a, vp_a) - this.m_ratio * b2_dot(this.m_u_b, vp_b);
	let impulse: f32 =-this.m_mass * cdot;
	this.m_impulse += impulse;

	let pa: B2vec2 =-impulse * this.m_u_a;
	let pb: B2vec2 =-this.m_ratio * impulse * this.m_u_b;
	v_a += this.m_inv_mass_a * pa;
	w_a += this.m_inv_ia * b2_cross(this.m_r_a, pa);
	v_b += this.m_inv_mass_b * pb;
	w_b += this.m_inv_ib * b2_cross(this.m_r_b, pb);

	velocities[this.m_index_a as usize] = B2velocity {v: v_a, w: w_a};
	velocities[this.m_index_b as usize] = B2velocity {v: v_b, w: w_b};
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	this: &B2pulleyJoint<D>,
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

	let r_a: B2vec2 =b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	let r_b: B2vec2 =b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);

	// Get the pulley axes.
	let mut u_a: B2vec2 =c_a + r_a - this.m_ground_anchor_a;
	let mut u_b: B2vec2 =c_b + r_b - this.m_ground_anchor_b;

	let length_a: f32 =u_a.length();
	let length_b: f32 =u_b.length();

	if length_a > 10.0 * B2_LINEAR_SLOP
	{
		u_a *= 1.0 / length_a;
	}
	else
	{
		u_a.set_zero();
	}

	if length_b > 10.0 * B2_LINEAR_SLOP
	{
		u_b *= 1.0 / length_b;
	}
	else
	{
		u_b.set_zero();
	}

	// Compute effective mass.
	let ru_a: f32 =b2_cross(r_a, u_a);
	let ru_b: f32 =b2_cross(r_b, u_b);

	let m_a: f32 =this.m_inv_mass_a + this.m_inv_ia * ru_a * ru_a;
	let m_b: f32 =this.m_inv_mass_b + this.m_inv_ib * ru_b * ru_b;

	let mut mass: f32 =m_a + this.m_ratio * this.m_ratio * m_b;

	if mass > 0.0
	{
		mass = 1.0 / mass;
	}

	let c: f32 = this.m_constant - length_a - this.m_ratio * length_b;
	let linear_error: f32 =b2_abs(c);

	let impulse: f32 =-mass * c;

	let pa: B2vec2 =-impulse * u_a;
	let pb: B2vec2 =-this.m_ratio * impulse * u_b;

	c_a += this.m_inv_mass_a * pa;
	a_a += this.m_inv_ia * b2_cross(r_a, pa);
	c_b += this.m_inv_mass_b * pb;
	a_b += this.m_inv_ib * b2_cross(r_b, pb);

	positions[this.m_index_a as usize] = B2position{c: c_a, a: a_a};
	positions[this.m_index_b as usize] = B2position{c: c_b, a: a_b};

	return linear_error < B2_LINEAR_SLOP;
}