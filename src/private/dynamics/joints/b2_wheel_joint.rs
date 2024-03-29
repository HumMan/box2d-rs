use crate::b2_draw::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_wheel_joint::*;

// Linear constraint (point-to-line)
// d = p_b - p_a = x_b + r_b - xA - r_a
// c = dot(ay, d)
// cdot = dot(d, cross(w_a, ay)) + dot(ay, v_b + cross(w_b, r_b) - v_a - cross(w_a, r_a))
//      = -dot(ay, v_a) - dot(cross(d + r_a, ay), w_a) + dot(ay, v_b) + dot(cross(r_b, ay), v_b)
// J = [-ay, -cross(d + r_a, ay), ay, cross(r_b, ay)]

// Spring linear constraint
// c = dot(ax, d)
// cdot = = -dot(ax, v_a) - dot(cross(d + r_a, ax), w_a) + dot(ax, v_b) + dot(cross(r_b, ax), v_b)
// J = [-ax -cross(d+r_a, ax) ax cross(r_b, ax)]

// Motor rotational constraint
// cdot = w_b - w_a
// J = [0 0 -1 0 0 1]

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	self_: &mut B2wheelJoint<D>,
	data: &B2solverData,
	positions: &[B2position],
	velocities: &mut [B2velocity],
) {
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

	let B2wheelJoint {
		m_inv_mass_a: m_a,
		m_inv_mass_b: m_b,
		m_inv_ia: i_a,
		m_inv_ib: i_b,
		..
	} = *self_;

	let B2position { c: c_a, a: a_a } = positions[self_.m_index_a as usize];
	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[self_.m_index_a as usize];

	let B2position { c: c_b, a: a_b } = positions[self_.m_index_b as usize];
	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[self_.m_index_b as usize];

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	// Compute the effective masses.
	let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
	let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);
	let d: B2vec2 = c_b + r_b - c_a - r_a;

	// Point to line constraint
	{
		self_.m_ay = b2_mul_rot_by_vec2(q_a, self_.m_local_yaxis_a);
		self_.m_s_ay = b2_cross(d + r_a, self_.m_ay);
		self_.m_s_by = b2_cross(r_b, self_.m_ay);

		self_.m_mass = m_a + m_b + i_a * self_.m_s_ay * self_.m_s_ay + i_b * self_.m_s_by * self_.m_s_by;

		if self_.m_mass > 0.0 {
			self_.m_mass = 1.0 / self_.m_mass;
		}
	}

	// Spring constraint
	self_.m_ax = b2_mul_rot_by_vec2(q_a, self_.m_local_xaxis_a);
	self_.m_s_ax = b2_cross(d + r_a, self_.m_ax);
	self_.m_s_bx = b2_cross(r_b, self_.m_ax);

	let inv_mass: f32 = m_a + m_b + i_a * self_.m_s_ax * self_.m_s_ax + i_b * self_.m_s_bx * self_.m_s_bx;
	if inv_mass > 0.0 {
		self_.m_axial_mass = 1.0 / inv_mass;
	} else {
		self_.m_axial_mass = 0.0;
	}

	self_.m_spring_mass = 0.0;
	self_.m_bias = 0.0;
	self_.m_gamma = 0.0;

	if self_.m_stiffness > 0.0 && inv_mass > 0.0 {
		self_.m_spring_mass = 1.0 / inv_mass;

		let c: f32 = b2_dot(d, self_.m_ax);

		// magic formulas
		let h: f32 = data.step.dt;
		self_.m_gamma = h * (self_.m_damping + h * self_.m_stiffness);
		if self_.m_gamma > 0.0 {
			self_.m_gamma = 1.0 / self_.m_gamma;
		}

		self_.m_bias = c * h * self_.m_stiffness * self_.m_gamma;

		self_.m_spring_mass = inv_mass + self_.m_gamma;
		if self_.m_spring_mass > 0.0 {
			self_.m_spring_mass = 1.0 / self_.m_spring_mass;
		}
	} else {
		self_.m_spring_impulse = 0.0;
	}

	if self_.m_enable_limit {
		self_.m_translation = b2_dot(self_.m_ax, d);
	} else {
		self_.m_lower_impulse = 0.0;
		self_.m_upper_impulse = 0.0;
	}

	if self_.m_enable_motor {
		self_.m_motor_mass = i_a + i_b;
		if self_.m_motor_mass > 0.0 {
			self_.m_motor_mass = 1.0 / self_.m_motor_mass;
		}
	} else {
		self_.m_motor_mass = 0.0;
		self_.m_motor_impulse = 0.0;
	}

	if data.step.warm_starting {
		// Account for variable time step.
		self_.m_impulse *= data.step.dt_ratio;
		self_.m_spring_impulse *= data.step.dt_ratio;
		self_.m_motor_impulse *= data.step.dt_ratio;

		let axial_impulse: f32 = self_.m_spring_impulse + self_.m_lower_impulse - self_.m_upper_impulse;
		let p: B2vec2 = self_.m_impulse * self_.m_ay + axial_impulse * self_.m_ax;
		let la: f32 = self_.m_impulse * self_.m_s_ay + axial_impulse * self_.m_s_ax + self_.m_motor_impulse;
		let lb: f32 = self_.m_impulse * self_.m_s_by + axial_impulse * self_.m_s_bx + self_.m_motor_impulse;

		v_a -= self_.m_inv_mass_a * p;
		w_a -= self_.m_inv_ia * la;

		v_b += self_.m_inv_mass_b * p;
		w_b += self_.m_inv_ib * lb;
	} else {
		self_.m_impulse = 0.0;
		self_.m_spring_impulse = 0.0;
		self_.m_motor_impulse = 0.0;
		self_.m_lower_impulse = 0.0;
		self_.m_upper_impulse = 0.0;
	}

	velocities[self_.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[self_.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	self_: &mut B2wheelJoint<D>,
	data: &B2solverData,
	velocities: &mut [B2velocity],
) {
	let m_a: f32 = self_.m_inv_mass_a;
	let m_b: f32 = self_.m_inv_mass_b;
	let i_a: f32 = self_.m_inv_ia;
	let i_b: f32 = self_.m_inv_ib;

	let mut v_a: B2vec2 = velocities[self_.m_index_a as usize].v;
	let mut w_a: f32 = velocities[self_.m_index_a as usize].w;
	let mut v_b: B2vec2 = velocities[self_.m_index_b as usize].v;
	let mut w_b: f32 = velocities[self_.m_index_b as usize].w;

	// solve spring constraint
	{
		let cdot: f32 = b2_dot(self_.m_ax, v_b - v_a) + self_.m_s_bx * w_b - self_.m_s_ax * w_a;
		let impulse: f32 =
			-self_.m_spring_mass * (cdot + self_.m_bias + self_.m_gamma * self_.m_spring_impulse);
		self_.m_spring_impulse += impulse;

		let p: B2vec2 = impulse * self_.m_ax;
		let la: f32 = impulse * self_.m_s_ax;
		let lb: f32 = impulse * self_.m_s_bx;

		v_a -= m_a * p;
		w_a -= i_a * la;

		v_b += m_b * p;
		w_b += i_b * lb;
	}

	// solve rotational motor constraint
	{
		let cdot: f32 = w_b - w_a - self_.m_motor_speed;
		let mut impulse: f32 = -self_.m_motor_mass * cdot;

		let old_impulse: f32 = self_.m_motor_impulse;
		let max_impulse: f32 = data.step.dt * self_.m_max_motor_torque;
		self_.m_motor_impulse = b2_clamp(self_.m_motor_impulse + impulse, -max_impulse, max_impulse);
		impulse = self_.m_motor_impulse - old_impulse;

		w_a -= i_a * impulse;
		w_b += i_b * impulse;
	}

	if self_.m_enable_limit {
		// Lower limit
		{
			let c: f32 = self_.m_translation - self_.m_lower_translation;
			let cdot: f32 = b2_dot(self_.m_ax, v_b - v_a) + self_.m_s_bx * w_b - self_.m_s_ax * w_a;
			let mut impulse: f32 = -self_.m_axial_mass * (cdot + b2_max(c, 0.0) * data.step.inv_dt);
			let old_impulse: f32 = self_.m_lower_impulse;
			self_.m_lower_impulse = b2_max(self_.m_lower_impulse + impulse, 0.0);
			impulse = self_.m_lower_impulse - old_impulse;

			let p: B2vec2 = impulse * self_.m_ax;
			let la: f32 = impulse * self_.m_s_ax;
			let lb: f32 = impulse * self_.m_s_bx;

			v_a -= m_a * p;
			w_a -= i_a * la;
			v_b += m_b * p;
			w_b += i_b * lb;
		}

		// Upper limit
		// Note: signs are flipped to keep c positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			let c: f32 = self_.m_upper_translation - self_.m_translation;
			let cdot: f32 = b2_dot(self_.m_ax, v_a - v_b) + self_.m_s_ax * w_a - self_.m_s_bx * w_b;
			let mut impulse: f32 = -self_.m_axial_mass * (cdot + b2_max(c, 0.0) * data.step.inv_dt);
			let old_impulse: f32 = self_.m_upper_impulse;
			self_.m_upper_impulse = b2_max(self_.m_upper_impulse + impulse, 0.0);
			impulse = self_.m_upper_impulse - old_impulse;

			let p: B2vec2 = impulse * self_.m_ax;
			let la: f32 = impulse * self_.m_s_ax;
			let lb: f32 = impulse * self_.m_s_bx;

			v_a += m_a * p;
			w_a += i_a * la;
			v_b -= m_b * p;
			w_b -= i_b * lb;
		}
	}

	// solve point to line constraint
	{
		let cdot: f32 = b2_dot(self_.m_ay, v_b - v_a) + self_.m_s_by * w_b - self_.m_s_ay * w_a;
		let impulse: f32 = -self_.m_mass * cdot;
		self_.m_impulse += impulse;

		let p: B2vec2 = impulse * self_.m_ay;
		let la: f32 = impulse * self_.m_s_ay;
		let lb: f32 = impulse * self_.m_s_by;

		v_a -= m_a * p;
		w_a -= i_a * la;

		v_b += m_b * p;
		w_b += i_b * lb;
	}

	velocities[self_.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[self_.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	self_: &B2wheelJoint<D>,
	_data: &B2solverData,
	positions: &mut [B2position],
) -> bool {
	let B2position {
		c: mut c_a,
		a: mut a_a,
	} = positions[self_.m_index_a as usize];
	let B2position {
		c: mut c_b,
		a: mut a_b,
	} = positions[self_.m_index_b as usize];

	let mut linear_error: f32 = 0.0;

	if self_.m_enable_limit {
		let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

		let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
		let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);
		let d: B2vec2 = (c_b - c_a) + r_b - r_a;

		let ax: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_xaxis_a);
		let s_ax: f32 = b2_cross(d + r_a, self_.m_ax);
		let s_bx: f32 = b2_cross(r_b, self_.m_ax);

		let mut c: f32 = 0.0;
		let translation: f32 = b2_dot(ax, d);
		if b2_abs(self_.m_upper_translation - self_.m_lower_translation) < 2.0 * B2_LINEAR_SLOP {
			c = translation;
		} else if translation <= self_.m_lower_translation {
			c = b2_min(translation - self_.m_lower_translation, 0.0);
		} else if translation >= self_.m_upper_translation {
			c = b2_max(translation - self_.m_upper_translation, 0.0);
		}

		if c != 0.0 {
			let inv_mass: f32 = self_.m_inv_mass_a
				+ self_.m_inv_mass_b
				+ self_.m_inv_ia * s_ax * s_ax
				+ self_.m_inv_ib * s_bx * s_bx;
			let mut impulse: f32 = 0.0;
			if inv_mass != 0.0 {
				impulse = -c / inv_mass;
			}

			let p: B2vec2 = impulse * ax;
			let la: f32 = impulse * s_ax;
			let lb: f32 = impulse * s_bx;

			c_a -= self_.m_inv_mass_a * p;
			a_a -= self_.m_inv_ia * la;
			c_b += self_.m_inv_mass_b * p;
			a_b += self_.m_inv_ib * lb;

			linear_error = b2_abs(c);
		}
	}

	// solve perpendicular constraint
	{
		let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

		let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
		let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);
		let d: B2vec2 = (c_b - c_a) + r_b - r_a;

		let ay: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_yaxis_a);

		let s_ay: f32 = b2_cross(d + r_a, ay);
		let s_by: f32 = b2_cross(r_b, ay);

		let c: f32 = b2_dot(d, ay);

		let inv_mass: f32 = self_.m_inv_mass_a
			+ self_.m_inv_mass_b
			+ self_.m_inv_ia * self_.m_s_ay * self_.m_s_ay
			+ self_.m_inv_ib * self_.m_s_by * self_.m_s_by;

		let mut impulse: f32 = 0.0;
		if inv_mass != 0.0 {
			impulse = -c / inv_mass;
		}

		let p: B2vec2 = impulse * ay;
		let la: f32 = impulse * s_ay;
		let lb: f32 = impulse * s_by;

		c_a -= self_.m_inv_mass_a * p;
		a_a -= self_.m_inv_ia * la;
		c_b += self_.m_inv_mass_b * p;
		a_b += self_.m_inv_ib * lb;

		linear_error = b2_max(linear_error, b2_abs(c));
	}

	positions[self_.m_index_a as usize] = B2position { c: c_a, a: a_a };
	positions[self_.m_index_b as usize] = B2position { c: c_b, a: a_b };

	return linear_error <= B2_LINEAR_SLOP;
}

// void B2wheelJoint::dump()
// {
// 	// FLT_DECIMAL_DIG == 9

// 	let index_a: i32 =m_body_a->m_island_index;
// 	let index_b: i32 =m_body_b->m_island_index;

// 	b2Log("  let jd = B2wheelJointDef::default();\n");
// 	b2Log("  jd.body_a = bodies[%d];\n", index_a);
// 	b2Log("  jd.body_b = bodies[%d];\n", index_b);
// 	b2Log("  jd.collide_connected = bool(%d);\n", m_collide_connected);
// 	b2Log("  jd.local_anchor_a.set(%.9g, %.9g);\n", m_local_anchor_a.x, m_local_anchor_a.y);
// 	b2Log("  jd.local_anchor_b.set(%.9g, %.9g);\n", m_local_anchor_b.x, m_local_anchor_b.y);
// 	b2Log("  jd.local_axis_a.set(%.9g, %.9g);\n", m_local_xaxis_a.x, m_local_xaxis_a.y);
// 	b2Log("  jd.enable_motor = bool(%d);\n", m_enable_motor);
// 	b2Log("  jd.motor_speed = %.9g;\n", m_motor_speed);
// 	b2Log("  jd.max_motor_torque = %.9g;\n", m_max_motor_torque);
// 	b2Log("  jd.stiffness = %.9g;\n", m_stiffness);
// 	b2Log("  jd.damping = %.9g;\n", m_damping);
// 	b2Log("  joints[%d] = m_world->create_joint(&jd);\n", m_index);
// }

///
pub(crate) fn draw<D: UserDataType>(self_: &B2wheelJoint<D>, draw: &mut dyn B2drawTrait) {
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
