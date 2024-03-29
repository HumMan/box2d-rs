use crate::b2_draw::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::joints::b2_revolute_joint::*;

// Point-to-point constraint
// c = p2 - p1
// cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-i -r1_skew i r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// k = invI1 + invI2

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	self_: &mut B2revoluteJoint<D>,
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

	let a_a: f32 = positions[self_.m_index_a as usize].a;
	let a_b: f32 = positions[self_.m_index_b as usize].a;

	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[self_.m_index_a as usize];

	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[self_.m_index_b as usize];
	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	self_.m_r_a = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
	self_.m_r_b = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);

	// J = [-i -r1_skew i r2_skew]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ m_a+r1y^2*i_a+m_b+r2y^2*i_b,  -r1y*i_a*r1x-r2y*i_b*r2x]
	//     [  -r1y*i_a*r1x-r2y*i_b*r2x, m_a+r1x^2*i_a+m_b+r2x^2*i_b]

	let m_a: f32 = self_.m_inv_mass_a;
	let m_b: f32 = self_.m_inv_mass_b;
	let i_a: f32 = self_.m_inv_ia;
	let i_b: f32 = self_.m_inv_ib;

	self_.m_k.ex.x = m_a + m_b + self_.m_r_a.y * self_.m_r_a.y * i_a + self_.m_r_b.y * self_.m_r_b.y * i_b;
	self_.m_k.ey.x = -self_.m_r_a.y * self_.m_r_a.x * i_a - self_.m_r_b.y * self_.m_r_b.x * i_b;
	self_.m_k.ex.y = self_.m_k.ey.x;
	self_.m_k.ey.y = m_a + m_b + self_.m_r_a.x * self_.m_r_a.x * i_a + self_.m_r_b.x * self_.m_r_b.x * i_b;

	self_.m_axial_mass = i_a + i_b;
	let fixed_rotation: bool;
	if self_.m_axial_mass > 0.0 {
		self_.m_axial_mass = 1.0 / self_.m_axial_mass;
		fixed_rotation = false;
	} else {
		fixed_rotation = true;
	}

	self_.m_angle = a_b - a_a - self_.m_reference_angle;
	if self_.m_enable_limit == false || fixed_rotation {
		self_.m_lower_impulse = 0.0;
		self_.m_upper_impulse = 0.0;
	}

	if self_.m_enable_motor == false || fixed_rotation {
		self_.m_motor_impulse = 0.0;
	}

	if data.step.warm_starting {
		// Scale impulses to support a variable time step.
		self_.m_impulse *= data.step.dt_ratio;
		self_.m_motor_impulse *= data.step.dt_ratio;

		self_.m_lower_impulse *= data.step.dt_ratio;
		self_.m_upper_impulse *= data.step.dt_ratio;

		let axial_impulse: f32 = self_.m_motor_impulse + self_.m_lower_impulse - self_.m_upper_impulse;
		let p = B2vec2::new(self_.m_impulse.x, self_.m_impulse.y);

		v_a -= m_a * p;
		w_a -= i_a * (b2_cross(self_.m_r_a, p) + axial_impulse);

		v_b += m_b * p;
		w_b += i_b * (b2_cross(self_.m_r_b, p) + axial_impulse);
	} else {
		self_.m_impulse.set_zero();
		self_.m_motor_impulse = 0.0;

		self_.m_lower_impulse = 0.0;
		self_.m_upper_impulse = 0.0;
	}

	velocities[self_.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[self_.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	self_: &mut B2revoluteJoint<D>,
	data: &B2solverData,
	velocities: &mut [B2velocity],
) {
	let B2velocity {
		v: mut v_a,
		w: mut w_a,
	} = velocities[self_.m_index_a as usize];

	let B2velocity {
		v: mut v_b,
		w: mut w_b,
	} = velocities[self_.m_index_b as usize];

	let B2revoluteJoint {
		m_inv_mass_a: m_a,
		m_inv_mass_b: m_b,
		m_inv_ia: i_a,
		m_inv_ib: i_b,
		..
	} = *self_;

	let fixed_rotation: bool = i_a + i_b == 0.0;

	// solve motor constraint.
	if self_.m_enable_motor && fixed_rotation == false {
		let cdot: f32 = w_b - w_a - self_.m_motor_speed;
		let mut impulse: f32 = -self_.m_axial_mass * cdot;
		let old_impulse: f32 = self_.m_motor_impulse;
		let max_impulse: f32 = data.step.dt * self_.m_max_motor_torque;
		self_.m_motor_impulse = b2_clamp(self_.m_motor_impulse + impulse, -max_impulse, max_impulse);
		impulse = self_.m_motor_impulse - old_impulse;

		w_a -= i_a * impulse;
		w_b += i_b * impulse;
	}

	if self_.m_enable_limit && fixed_rotation == false {
		// Lower limit
		{
			let c: f32 = self_.m_angle - self_.m_lower_angle;
			let cdot: f32 = w_b - w_a;
			let mut impulse: f32 = -self_.m_axial_mass * (cdot + b2_max(c, 0.0) * data.step.inv_dt);
			let old_impulse: f32 = self_.m_lower_impulse;
			self_.m_lower_impulse = b2_max(self_.m_lower_impulse + impulse, 0.0);
			impulse = self_.m_lower_impulse - old_impulse;

			w_a -= i_a * impulse;
			w_b += i_b * impulse;
		}

		// Upper limit
		// Note: signs are flipped to keep c positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			let c: f32 = self_.m_upper_angle - self_.m_angle;
			let cdot: f32 = w_a - w_b;
			let mut impulse: f32 = -self_.m_axial_mass * (cdot + b2_max(c, 0.0) * data.step.inv_dt);
			let old_impulse: f32 = self_.m_upper_impulse;
			self_.m_upper_impulse = b2_max(self_.m_upper_impulse + impulse, 0.0);
			impulse = self_.m_upper_impulse - old_impulse;

			w_a += i_a * impulse;
			w_b -= i_b * impulse;
		}
	}

	{
		// solve point-to-point constraint
		let cdot: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b)
			- v_a - b2_cross_scalar_by_vec(w_a, self_.m_r_a);
		let impulse: B2vec2 = self_.m_k.solve(-cdot);

		self_.m_impulse.x += impulse.x;
		self_.m_impulse.y += impulse.y;

		v_a -= m_a * impulse;
		w_a -= i_a * b2_cross(self_.m_r_a, impulse);

		v_b += m_b * impulse;
		w_b += i_b * b2_cross(self_.m_r_b, impulse);
	}

	//TODO_humman - optimize others like this
	velocities[self_.m_index_a as usize] = B2velocity { v: v_a, w: w_a };
	velocities[self_.m_index_b as usize] = B2velocity { v: v_b, w: w_b };
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	self_: &B2revoluteJoint<D>,
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

	let (mut q_a, mut q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	let mut angular_error: f32 = 0.0;
	let position_error: f32;

	let fixed_rotation: bool = self_.m_inv_ia + self_.m_inv_ib == 0.0;

	// Solve angular limit constraint
	if self_.m_enable_limit && fixed_rotation == false {
		let angle: f32 = a_b - a_a - self_.m_reference_angle;
		let mut c: f32 = 0.0;

		if b2_abs(self_.m_upper_angle - self_.m_lower_angle) < 2.0 * B2_ANGULAR_SLOP {
			// Prevent large angular corrections
			c = b2_clamp(
				angle - self_.m_lower_angle,
				-B2_MAX_ANGULAR_CORRECTION,
				B2_MAX_ANGULAR_CORRECTION,
			);
		} else if angle <= self_.m_lower_angle {
			// Prevent large angular corrections and allow some slop.
			c = b2_clamp(
				angle - self_.m_lower_angle + B2_ANGULAR_SLOP,
				-B2_MAX_ANGULAR_CORRECTION,
				0.0,
			);
		} else if angle >= self_.m_upper_angle {
			// Prevent large angular corrections and allow some slop.
			c = b2_clamp(
				angle - self_.m_upper_angle - B2_ANGULAR_SLOP,
				0.0,
				B2_MAX_ANGULAR_CORRECTION,
			);
		}

		let limit_impulse: f32 = -self_.m_axial_mass * c;
		a_a -= self_.m_inv_ia * limit_impulse;
		a_b += self_.m_inv_ib * limit_impulse;
		angular_error = b2_abs(c);
	}

	// solve point-to-point constraint.
	{
		q_a.set(a_a);
		q_b.set(a_b);
		let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
		let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);

		let c: B2vec2 = c_b + r_b - c_a - r_a;
		position_error = c.length();

		let m_a: f32 = self_.m_inv_mass_a;
		let m_b: f32 = self_.m_inv_mass_b;
		let i_a: f32 = self_.m_inv_ia;
		let i_b: f32 = self_.m_inv_ib;

		let mut k = B2Mat22::default();
		k.ex.x = m_a + m_b + i_a * r_a.y * r_a.y + i_b * r_b.y * r_b.y;
		k.ex.y = -i_a * r_a.x * r_a.y - i_b * r_b.x * r_b.y;
		k.ey.x = k.ex.y;
		k.ey.y = m_a + m_b + i_a * r_a.x * r_a.x + i_b * r_b.x * r_b.x;

		let impulse: B2vec2 = -k.solve(c);

		c_a -= m_a * impulse;
		a_a -= i_a * b2_cross(r_a, impulse);

		c_b += m_b * impulse;
		a_b += i_b * b2_cross(r_b, impulse);
	}

	positions[self_.m_index_a as usize] = B2position { c: c_a, a: a_a };
	positions[self_.m_index_b as usize] = B2position { c: c_b, a: a_b };
	return position_error <= B2_LINEAR_SLOP && angular_error <= B2_ANGULAR_SLOP;
}

pub(crate) fn draw<D: UserDataType>(self_: &B2revoluteJoint<D>, draw: &mut dyn B2drawTrait) {
	let xf_a = self_.base.m_body_a.borrow().get_transform();
	let xf_b = self_.base.m_body_b.borrow().get_transform();
	let p_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, self_.m_local_anchor_a);
	let p_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, self_.m_local_anchor_b);

	let c1 = B2color::new(0.7, 0.7, 0.7);
	let c2 = B2color::new(0.3, 0.9, 0.3);
	let c3 = B2color::new(0.9, 0.3, 0.3);
	let c4 = B2color::new(0.3, 0.3, 0.9);
	let c5 = B2color::new(0.4, 0.4, 0.4);

	draw.draw_point(p_a, 5.0, c4);
	draw.draw_point(p_b, 5.0, c5);

	let a_a: f32 = self_.base.m_body_a.borrow().get_angle();
	let a_b: f32 = self_.base.m_body_b.borrow().get_angle();
	let angle: f32 = a_b - a_a - self_.m_reference_angle;

	const L: f32 = 0.5;

	let r: B2vec2 = L * B2vec2::new(f32::cos(angle), f32::sin(angle));
	draw.draw_segment(p_b, p_b + r, c1);
	draw.draw_circle(p_b, L, c1);

	if self_.m_enable_limit {
		let rlo: B2vec2 =
			L * B2vec2::new(f32::cos(self_.m_lower_angle), f32::sin(self_.m_lower_angle));
		let rhi: B2vec2 =
			L * B2vec2::new(f32::cos(self_.m_upper_angle), f32::sin(self_.m_upper_angle));

		draw.draw_segment(p_b, p_b + rlo, c2);
		draw.draw_segment(p_b, p_b + rhi, c3);
	}

	let color = B2color::new(0.5, 0.8, 0.8);
	draw.draw_segment(xf_a.p, p_a, color);
	draw.draw_segment(p_a, p_b, color);
	draw.draw_segment(xf_b.p, p_b, color);
}
