use crate::b2_draw::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_prismatic_joint::*;

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// c = dot(perp, d)
// cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// c = a2 - a1 + a_initial
// cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// k = J * inv_m * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// c = dot(ax1, d)
// cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want c2 = c1 + h * cdot >= 0
// Or:
// cdot + c1/h >= 0
// i do not apply a negative constraint error because that is handled in position correction.
// So:
// cdot + max(c1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

pub(crate) fn new<D: UserDataType>(def: &B2prismaticJointDef<D>) -> B2prismaticJoint<D> {
	let m_local_anchor_a = def.local_anchor_a;
	let m_local_anchor_b = def.local_anchor_b;
	let mut m_local_xaxis_a = def.local_axis_a;
	m_local_xaxis_a.normalize();
	let m_local_yaxis_a = b2_cross_scalar_by_vec(1.0, m_local_xaxis_a);
	let m_reference_angle = def.reference_angle;

	let m_impulse = B2vec2::zero();
	//let m_axial_mass = 0.0;
	let m_motor_impulse = 0.0;
	let m_lower_impulse = 0.0;
	let m_upper_impulse = 0.0;

	let m_lower_translation = def.lower_translation;
	let m_upper_translation = def.upper_translation;

	b2_assert(m_lower_translation <= m_upper_translation);

	let m_max_motor_force = def.max_motor_force;
	let m_motor_speed = def.motor_speed;
	let m_enable_limit = def.enable_limit;
	let m_enable_motor = def.enable_motor;

	let m_axis = B2vec2::zero();
	let m_perp = B2vec2::zero();

	return B2prismaticJoint {
		base: B2joint::new(&def.base),

		m_local_anchor_a,
		m_local_anchor_b,
		m_local_xaxis_a,
		m_local_yaxis_a,
		m_reference_angle,
		m_impulse,
		m_motor_impulse,
		m_lower_impulse,
		m_upper_impulse,
		m_lower_translation,
		m_upper_translation,
		m_max_motor_force,
		m_motor_speed,
		m_enable_limit,
		m_enable_motor,

		m_index_a: 0,
		m_index_b: 0,
		m_local_center_a: B2vec2::zero(),
		m_local_center_b: B2vec2::zero(),
		m_inv_mass_a: 0.0,
		m_inv_mass_b: 0.0,
		m_inv_ia: 0.0,
		m_inv_ib: 0.0,
		m_axis,
		m_perp,
		m_s1: 0.0,
		m_s2: 0.0,
		m_a1: 0.0,
		m_a2: 0.0,
		m_k: B2Mat22::default(),
		m_translation: 0.0,
		m_axial_mass: 0.0,
	};
}

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	this: &mut B2prismaticJoint<D>,
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

	let c_a: B2vec2 = positions[this.m_index_a as usize].c;
	let a_a: f32 = positions[this.m_index_a as usize].a;
	let mut v_a: B2vec2 = velocities[this.m_index_a as usize].v;
	let mut w_a: f32 = velocities[this.m_index_a as usize].w;

	let c_b: B2vec2 = positions[this.m_index_b as usize].c;
	let a_b: f32 = positions[this.m_index_b as usize].a;
	let mut v_b: B2vec2 = velocities[this.m_index_b as usize].v;
	let mut w_b: f32 = velocities[this.m_index_b as usize].w;

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	// Compute the effective masses.
	let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);
	let d: B2vec2 = (c_b - c_a) + r_b - r_a;

	let m_a: f32 = this.m_inv_mass_a;
	let m_b: f32 = this.m_inv_mass_b;
	let i_a: f32 = this.m_inv_ia;
	let i_b: f32 = this.m_inv_ib;

	// Compute motor Jacobian and effective mass.
	{
		this.m_axis = b2_mul_rot_by_vec2(q_a, this.m_local_xaxis_a);
		this.m_a1 = b2_cross(d + r_a, this.m_axis);
		this.m_a2 = b2_cross(r_b, this.m_axis);

		this.m_axial_mass = m_a + m_b + i_a * this.m_a1 * this.m_a1 + i_b * this.m_a2 * this.m_a2;
		if this.m_axial_mass > 0.0 {
			this.m_axial_mass = 1.0 / this.m_axial_mass;
		}
	}

	// Prismatic constraint.
	{
		this.m_perp = b2_mul_rot_by_vec2(q_a, this.m_local_yaxis_a);

		this.m_s1 = b2_cross(d + r_a, this.m_perp);
		this.m_s2 = b2_cross(r_b, this.m_perp);

		let k11: f32 = m_a + m_b + i_a * this.m_s1 * this.m_s1 + i_b * this.m_s2 * this.m_s2;
		let k12: f32 = i_a * this.m_s1 + i_b * this.m_s2;
		let mut k22: f32 = i_a + i_b;
		if k22 == 0.0 {
			// For bodies with fixed rotation.
			k22 = 1.0;
		}

		this.m_k.ex.set(k11, k12);
		this.m_k.ey.set(k12, k22);
	}

	if this.m_enable_limit {
		this.m_translation = b2_dot(this.m_axis, d);
	} else {
		this.m_lower_impulse = 0.0;
		this.m_upper_impulse = 0.0;
	}

	if this.m_enable_motor == false {
		this.m_motor_impulse = 0.0;
	}

	if data.step.warm_starting {
		// Account for variable time step.
		this.m_impulse *= data.step.dt_ratio;
		this.m_motor_impulse *= data.step.dt_ratio;
		this.m_lower_impulse *= data.step.dt_ratio;
		this.m_upper_impulse *= data.step.dt_ratio;

		let axial_impulse: f32 = this.m_motor_impulse + this.m_lower_impulse - this.m_upper_impulse;
		let p: B2vec2 = this.m_impulse.x * this.m_perp + axial_impulse * this.m_axis;
		let la: f32 = this.m_impulse.x * this.m_s1 + this.m_impulse.y + axial_impulse * this.m_a1;
		let lb: f32 = this.m_impulse.x * this.m_s2 + this.m_impulse.y + axial_impulse * this.m_a2;

		v_a -= m_a * p;
		w_a -= i_a * la;

		v_b += m_b * p;
		w_b += i_b * lb;
	} else {
		this.m_impulse.set_zero();
		this.m_motor_impulse = 0.0;
		this.m_lower_impulse = 0.0;
		this.m_upper_impulse = 0.0;
	}

	velocities[this.m_index_a as usize].v = v_a;
	velocities[this.m_index_a as usize].w = w_a;
	velocities[this.m_index_b as usize].v = v_b;
	velocities[this.m_index_b as usize].w = w_b;
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	this: &mut B2prismaticJoint<D>,
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

	// solve linear motor constraint
	if this.m_enable_motor {
		let cdot: f32 = b2_dot(this.m_axis, v_b - v_a) + this.m_a2 * w_b - this.m_a1 * w_a;
		let mut impulse: f32 = this.m_axial_mass * (this.m_motor_speed - cdot);
		let old_impulse: f32 = this.m_motor_impulse;
		let max_impulse: f32 = data.step.dt * this.m_max_motor_force;
		this.m_motor_impulse = b2_clamp(this.m_motor_impulse + impulse, -max_impulse, max_impulse);
		impulse = this.m_motor_impulse - old_impulse;

		let p: B2vec2 = impulse * this.m_axis;
		let la: f32 = impulse * this.m_a1;
		let lb: f32 = impulse * this.m_a2;

		v_a -= m_a * p;
		w_a -= i_a * la;
		v_b += m_b * p;
		w_b += i_b * lb;
	}

	if this.m_enable_limit {
		// Lower limit
		{
			let c: f32 = this.m_translation - this.m_lower_translation;
			let cdot: f32 = b2_dot(this.m_axis, v_b - v_a) + this.m_a2 * w_b - this.m_a1 * w_a;
			let mut impulse: f32 = -this.m_axial_mass * (cdot + b2_max(c, 0.0) * data.step.inv_dt);
			let old_impulse: f32 = this.m_lower_impulse;
			this.m_lower_impulse = b2_max(this.m_lower_impulse + impulse, 0.0);
			impulse = this.m_lower_impulse - old_impulse;

			let p: B2vec2 = impulse * this.m_axis;
			let la: f32 = impulse * this.m_a1;
			let lb: f32 = impulse * this.m_a2;

			v_a -= m_a * p;
			w_a -= i_a * la;
			v_b += m_b * p;
			w_b += i_b * lb;
		}

		// Upper limit
		// Note: signs are flipped to keep c positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			let c: f32 = this.m_upper_translation - this.m_translation;
			let cdot: f32 = b2_dot(this.m_axis, v_a - v_b) + this.m_a1 * w_a - this.m_a2 * w_b;
			let mut impulse: f32 = -this.m_axial_mass * (cdot + b2_max(c, 0.0) * data.step.inv_dt);
			let old_impulse: f32 = this.m_upper_impulse;
			this.m_upper_impulse = b2_max(this.m_upper_impulse + impulse, 0.0);
			impulse = this.m_upper_impulse - old_impulse;

			let p: B2vec2 = impulse * this.m_axis;
			let la: f32 = impulse * this.m_a1;
			let lb: f32 = impulse * this.m_a2;

			v_a += m_a * p;
			w_a += i_a * la;
			v_b -= m_b * p;
			w_b -= i_b * lb;
		}
	}

	// solve the prismatic constraint in block form.
	{
		let cdot = B2vec2 {
			x: b2_dot(this.m_perp, v_b - v_a) + this.m_s2 * w_b - this.m_s1 * w_a,
			y: w_b - w_a,
		};

		let df: B2vec2 = this.m_k.solve(-cdot);
		this.m_impulse += df;

		let p: B2vec2 = df.x * this.m_perp;
		let la: f32 = df.x * this.m_s1 + df.y;
		let lb: f32 = df.x * this.m_s2 + df.y;

		v_a -= m_a * p;
		w_a -= i_a * la;

		v_b += m_b * p;
		w_b += i_b * lb;
	}

	velocities[this.m_index_a as usize].v = v_a;
	velocities[this.m_index_a as usize].w = w_a;
	velocities[this.m_index_b as usize].v = v_b;
	velocities[this.m_index_b as usize].w = w_b;
}

// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
pub(crate) fn solve_position_constraints<D: UserDataType>(
	this: &B2prismaticJoint<D>,
	_data: &mut B2solverData,
	positions: &mut [B2position],
) -> bool {
	let mut c_a: B2vec2 = positions[this.m_index_a as usize].c;
	let mut a_a: f32 = positions[this.m_index_a as usize].a;
	let mut c_b: B2vec2 = positions[this.m_index_b as usize].c;
	let mut a_b: f32 = positions[this.m_index_b as usize].a;

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	let m_a: f32 = this.m_inv_mass_a;
	let m_b: f32 = this.m_inv_mass_b;
	let i_a: f32 = this.m_inv_ia;
	let i_b: f32 = this.m_inv_ib;

	// Compute fresh Jacobians
	let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);
	let d: B2vec2 = c_b + r_b - c_a - r_a;

	let axis: B2vec2 = b2_mul_rot_by_vec2(q_a, this.m_local_xaxis_a);
	let a1: f32 = b2_cross(d + r_a, axis);
	let a2: f32 = b2_cross(r_b, axis);
	let perp: B2vec2 = b2_mul_rot_by_vec2(q_a, this.m_local_yaxis_a);

	let s1: f32 = b2_cross(d + r_a, perp);
	let s2: f32 = b2_cross(r_b, perp);

	let impulse: B2Vec3;
	let c1 = B2vec2 {
		x: b2_dot(perp, d),
		y: a_b - a_a - this.m_reference_angle,
	};

	let mut linear_error: f32 = b2_abs(c1.x);
	let angular_error: f32 = b2_abs(c1.y);

	let mut active: bool = false;
	let mut c2: f32 = 0.0;
	if this.m_enable_limit {
		let translation: f32 = b2_dot(axis, d);
		if b2_abs(this.m_upper_translation - this.m_lower_translation) < 2.0 * B2_LINEAR_SLOP {
			c2 = translation;
			linear_error = b2_max(linear_error, b2_abs(translation));
			active = true;
		} else if translation <= this.m_lower_translation {
			c2 = b2_min(translation - this.m_lower_translation, 0.0);
			linear_error = b2_max(linear_error, this.m_lower_translation - translation);
			active = true;
		} else if translation >= this.m_upper_translation {
			c2 = b2_max(translation - this.m_upper_translation, 0.0);
			linear_error = b2_max(linear_error, translation - this.m_upper_translation);
			active = true;
		}
	}

	if active {
		let k11: f32 = m_a + m_b + i_a * s1 * s1 + i_b * s2 * s2;
		let k12: f32 = i_a * s1 + i_b * s2;
		let k13: f32 = i_a * s1 * a1 + i_b * s2 * a2;
		let mut k22: f32 = i_a + i_b;
		if k22 == 0.0 {
			// For fixed rotation
			k22 = 1.0;
		}
		let k23: f32 = i_a * a1 + i_b * a2;
		let k33: f32 = m_a + m_b + i_a * a1 * a1 + i_b * a2 * a2;

		let k = B2Mat33 {
			ex: B2Vec3::new(k11, k12, k13),
			ey: B2Vec3::new(k12, k22, k23),
			ez: B2Vec3::new(k13, k23, k33),
		};

		let c = B2Vec3 {
			x: c1.x,
			y: c1.y,
			z: c2,
		};

		impulse = k.solve33(-c);
	} else {
		let k11: f32 = m_a + m_b + i_a * s1 * s1 + i_b * s2 * s2;
		let k12: f32 = i_a * s1 + i_b * s2;
		let mut k22: f32 = i_a + i_b;
		if k22 == 0.0 {
			k22 = 1.0;
		}

		let k = B2Mat22 {
			ex: B2vec2::new(k11, k12),
			ey: B2vec2::new(k12, k22),
		};

		let impulse1: B2vec2 = k.solve(-c1);
		impulse = B2Vec3 {
			x: impulse1.x,
			y: impulse1.y,
			z: 0.0,
		};
	}

	let p: B2vec2 = impulse.x * perp + impulse.z * axis;
	let la: f32 = impulse.x * s1 + impulse.y + impulse.z * a1;
	let lb: f32 = impulse.x * s2 + impulse.y + impulse.z * a2;

	c_a -= m_a * p;
	a_a -= i_a * la;
	c_b += m_b * p;
	a_b += i_b * lb;

	positions[this.m_index_a as usize].c = c_a;
	positions[this.m_index_a as usize].a = a_a;
	positions[this.m_index_b as usize].c = c_b;
	positions[this.m_index_b as usize].a = a_b;

	return linear_error <= B2_LINEAR_SLOP && angular_error <= B2_ANGULAR_SLOP;
}

pub(crate) fn get_joint_translation<D: UserDataType>(this: &B2prismaticJoint<D>) -> f32 {
	let p_a: B2vec2 = this
		.base
		.m_body_a
		.borrow()
		.get_world_point(this.m_local_anchor_a);
	let p_b: B2vec2 = this
		.base
		.m_body_b
		.borrow()
		.get_world_point(this.m_local_anchor_b);
	let d: B2vec2 = p_b - p_a;
	let axis: B2vec2 = this
		.base
		.m_body_a
		.borrow()
		.get_world_vector(this.m_local_xaxis_a);

	let translation: f32 = b2_dot(d, axis);
	return translation;
}

pub(crate) fn get_joint_speed<D: UserDataType>(this: &B2prismaticJoint<D>) -> f32 {
	let b_a = this.base.m_body_a.borrow();
	let b_b = this.base.m_body_b.borrow();

	let r_a: B2vec2 =
		b2_mul_rot_by_vec2(b_a.m_xf.q, this.m_local_anchor_a - b_a.m_sweep.local_center);
	let r_b: B2vec2 =
		b2_mul_rot_by_vec2(b_b.m_xf.q, this.m_local_anchor_b - b_b.m_sweep.local_center);
	let p1: B2vec2 = b_a.m_sweep.c + r_a;
	let p2: B2vec2 = b_b.m_sweep.c + r_b;
	let d: B2vec2 = p2 - p1;
	let axis: B2vec2 = b2_mul_rot_by_vec2(b_a.m_xf.q, this.m_local_xaxis_a);

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

pub(crate) fn draw<D: UserDataType>(self_: &B2prismaticJoint<D>, draw: &mut dyn B2drawTrait) {
	let xf_a = self_.base.m_body_a.borrow().get_transform();
	let xf_b = self_.base.m_body_b.borrow().get_transform();
	let p_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, self_.m_local_anchor_a);
	let p_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, self_.m_local_anchor_b);

	let axis: B2vec2 = b2_mul_rot_by_vec2(xf_a.q, self_.m_local_xaxis_a);

	let c1 = B2color::new(0.7, 0.7, 0.7);
	let c2 = B2color::new(0.3, 0.9, 0.3);
	let c3 = B2color::new(0.9, 0.3, 0.3);
	let c4 = B2color::new(0.3, 0.3, 0.9);
	let c5 = B2color::new(0.4, 0.4, 0.4);

	draw.draw_segment(p_a, p_b, c5);

	if self_.m_enable_limit {
		let lower: B2vec2 = p_a + self_.m_lower_translation * axis;
		let upper: B2vec2 = p_a + self_.m_upper_translation * axis;
		let perp: B2vec2 = b2_mul_rot_by_vec2(xf_a.q, self_.m_local_yaxis_a);
		draw.draw_segment(lower, upper, c1);
		draw.draw_segment(lower - 0.5 * perp, lower + 0.5 * perp, c2);
		draw.draw_segment(upper - 0.5 * perp, upper + 0.5 * perp, c3);
	} else {
		draw.draw_segment(p_a - 1.0 * axis, p_a + 1.0 * axis, c1);
	}

	draw.draw_point(p_a, 5.0, c1);
	draw.draw_point(p_b, 5.0, c4);
}
