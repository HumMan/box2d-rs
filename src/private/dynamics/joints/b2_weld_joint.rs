use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_weld_joint::*;

// Point-to-point constraint
// c = p2 - p1
// cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-i -r1_skew i r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// c = angle2 - angle1 - reference_angle
// cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// k = invI1 + invI2

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	this: &mut B2weldJoint<D>,
	data: &B2solverData,
	positions: &[B2position],
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

	let a_a: f32 = positions[this.m_index_a as usize].a;
	let a_b: f32 = positions[this.m_index_b as usize].a;

	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[this.m_index_a as usize];

	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[this.m_index_b as usize];

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

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

	let mut k = B2Mat33::default();
	k.ex.x = m_a + m_b + this.m_r_a.y * this.m_r_a.y * i_a + this.m_r_b.y * this.m_r_b.y * i_b;
	k.ey.x = -this.m_r_a.y * this.m_r_a.x * i_a - this.m_r_b.y * this.m_r_b.x * i_b;
	k.ez.x = -this.m_r_a.y * i_a - this.m_r_b.y * i_b;
	k.ex.y = k.ey.x;
	k.ey.y = m_a + m_b + this.m_r_a.x * this.m_r_a.x * i_a + this.m_r_b.x * this.m_r_b.x * i_b;
	k.ez.y = this.m_r_a.x * i_a + this.m_r_b.x * i_b;
	k.ex.z = k.ez.x;
	k.ey.z = k.ez.y;
	k.ez.z = i_a + i_b;

	if this.m_stiffness > 0.0 {
		k.get_inverse22(&mut this.m_mass);
		let mut inv_m: f32 = i_a + i_b;

		let c: f32 = a_b - a_a - this.m_reference_angle;


		// Damping coefficient
		let d: f32 = this.m_damping;

		// Spring stiffness
		let k: f32 = this.m_stiffness;

		// magic formulas
		let h: f32 = data.step.dt;
		this.m_gamma = h * (d + h * k);
		this.m_gamma = if this.m_gamma != 0.0 {
			1.0 / this.m_gamma
		} else {
			0.0
		};
		this.m_bias = c * h * k * this.m_gamma;

		inv_m += this.m_gamma;
		this.m_mass.ez.z = if inv_m != 0.0 { 1.0 / inv_m } else { 0.0 };
	} else if k.ez.z == 0.0 {
		k.get_inverse22(&mut this.m_mass);
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
	} else {
		k.get_sym_inverse33(&mut this.m_mass);
		this.m_gamma = 0.0;
		this.m_bias = 0.0;
	}

	if data.step.warm_starting {
		// Scale impulses to support a variable time step.
		this.m_impulse *= data.step.dt_ratio;

		let p = B2vec2::new(this.m_impulse.x, this.m_impulse.y);

		v_a -= m_a * p;
		w_a -= i_a * (b2_cross(this.m_r_a, p) + this.m_impulse.z);

		v_b += m_b * p;
		w_b += i_b * (b2_cross(this.m_r_b, p) + this.m_impulse.z);
	} else {
		this.m_impulse.set_zero();
	}

	velocities[this.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[this.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	this: &mut B2weldJoint<D>,
	_data: &B2solverData,
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

	let B2weldJoint {
		m_inv_mass_a: m_a,
		m_inv_mass_b: m_b,
		m_inv_ia: i_a,
		m_inv_ib: i_b,
		..
	} = *this;

	if this.m_stiffness > 0.0 {
		let cdot2: f32 = w_b - w_a;

		let impulse2: f32 =
			-this.m_mass.ez.z * (cdot2 + this.m_bias + this.m_gamma * this.m_impulse.z);
		this.m_impulse.z += impulse2;

		w_a -= i_a * impulse2;
		w_b += i_b * impulse2;

		let cdot1: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, this.m_r_b)
			- v_a - b2_cross_scalar_by_vec(w_a, this.m_r_a);

		let impulse1: B2vec2 = -b2_mul22(this.m_mass, cdot1);
		this.m_impulse.x += impulse1.x;
		this.m_impulse.y += impulse1.y;

		let p: B2vec2 = impulse1;

		v_a -= m_a * p;
		w_a -= i_a * b2_cross(this.m_r_a, p);

		v_b += m_b * p;
		w_b += i_b * b2_cross(this.m_r_b, p);
	} else {
		let cdot1: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, this.m_r_b)
			- v_a - b2_cross_scalar_by_vec(w_a, this.m_r_a);
		let cdot2: f32 = w_b - w_a;
		let cdot = B2Vec3::new(cdot1.x, cdot1.y, cdot2);

		let impulse: B2Vec3 = -b2_mul_mat33(this.m_mass, cdot);
		this.m_impulse += impulse;

		let p = B2vec2::new(impulse.x, impulse.y);

		v_a -= m_a * p;
		w_a -= i_a * (b2_cross(this.m_r_a, p) + impulse.z);

		v_b += m_b * p;
		w_b += i_b * (b2_cross(this.m_r_b, p) + impulse.z);
	}

	velocities[this.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[this.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	this: &B2weldJoint<D>,
	_data: &B2solverData,
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

	let B2weldJoint {
		m_inv_mass_a: m_a,
		m_inv_mass_b: m_b,
		m_inv_ia: i_a,
		m_inv_ib: i_b,
		..
	} = *this;

	let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, this.m_local_anchor_a - this.m_local_center_a);
	let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, this.m_local_anchor_b - this.m_local_center_b);

	let position_error: f32;
	let angular_error: f32;

	let mut k = B2Mat33::default();
	k.ex.x = m_a + m_b + r_a.y * r_a.y * i_a + r_b.y * r_b.y * i_b;
	k.ey.x = -r_a.y * r_a.x * i_a - r_b.y * r_b.x * i_b;
	k.ez.x = -r_a.y * i_a - r_b.y * i_b;
	k.ex.y = k.ey.x;
	k.ey.y = m_a + m_b + r_a.x * r_a.x * i_a + r_b.x * r_b.x * i_b;
	k.ez.y = r_a.x * i_a + r_b.x * i_b;
	k.ex.z = k.ez.x;
	k.ey.z = k.ez.y;
	k.ez.z = i_a + i_b;

	if this.m_stiffness > 0.0 {
		let c1: B2vec2 = c_b + r_b - c_a - r_a;

		position_error = c1.length();
		angular_error = 0.0;

		let p: B2vec2 = -k.solve22(c1);

		c_a -= m_a * p;
		a_a -= i_a * b2_cross(r_a, p);

		c_b += m_b * p;
		a_b += i_b * b2_cross(r_b, p);
	} else {
		let c1: B2vec2 = c_b + r_b - c_a - r_a;
		let c2: f32 = a_b - a_a - this.m_reference_angle;

		position_error = c1.length();
		angular_error = b2_abs(c2);

		let c = B2Vec3::new(c1.x, c1.y, c2);
		let impulse: B2Vec3;
		if k.ez.z > 0.0 {
			impulse = -k.solve33(c);
		} else {
			let impulse2: B2vec2 = -k.solve22(c1);
			impulse = B2Vec3::new(impulse2.x, impulse2.y, 0.0);
		}

		let p = B2vec2::new(impulse.x, impulse.y);

		c_a -= m_a * p;
		a_a -= i_a * (b2_cross(r_a, p) + impulse.z);

		c_b += m_b * p;
		a_b += i_b * (b2_cross(r_b, p) + impulse.z);
	}

	positions[this.m_index_a as usize] = B2position { c: c_a, a: a_a };
	positions[this.m_index_b as usize] = B2position { c: c_b, a: a_b };

	return position_error <= B2_LINEAR_SLOP && angular_error <= B2_ANGULAR_SLOP;
}

// void B2weldJoint::dump()
// {
// 	let index_a: i32 =m_body_a->m_island_index;
// 	let index_b: i32 =m_body_b->m_island_index;

// 	b2Log("  B2weldJointDef jd;\n");
// 	b2Log("  jd.body_a = bodies[%d];\n", index_a);
// 	b2Log("  jd.body_b = bodies[%d];\n", index_b);
// 	b2Log("  jd.collide_connected = bool(%d);\n", m_collide_connected);
// 	b2Log("  jd.local_anchor_a.set(%.15lef, %.15lef);\n", m_local_anchor_a.x, m_local_anchor_a.y);
// 	b2Log("  jd.local_anchor_b.set(%.15lef, %.15lef);\n", m_local_anchor_b.x, m_local_anchor_b.y);
// 	b2Log("  jd.reference_angle = %.15lef;\n", m_reference_angle);
// 	b2Log("  jd.frequency_hz = %.15lef;\n", m_frequency_hz);
// 	b2Log("  jd.damping_ratio = %.15lef;\n", m_damping_ratio);
// 	b2Log("  joints[%d] = m_world->create_joint(&jd);\n", m_index);
// }
