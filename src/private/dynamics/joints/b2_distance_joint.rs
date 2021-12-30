use crate::b2_body::*;
use crate::b2_common::*;
use crate::b2_draw::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_time_step::*;
use crate::b2rs_common::UserDataType;
use crate::joints::b2_distance_joint::*;

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// c = norm(p2 - p1) - l
// u = (p2 - p1) / norm(p2 - p1)
// cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// k = J * inv_m * JT
// //   = inv_mass1 + invI1 * cross(r1, u)^2 + inv_mass2 + invI2 * cross(r2, u)^2

pub(crate) fn b2_distance_joint_def_initialize<D: UserDataType>(
	self_: &mut B2distanceJointDef<D>,
	b1: BodyPtr<D>,
	b2: BodyPtr<D>,
	anchor1: B2vec2,
	anchor2: B2vec2,
) {
	self_.local_anchor_a = b1.borrow().get_local_point(anchor1);
	self_.local_anchor_b = b2.borrow().get_local_point(anchor2);
	self_.base.body_a = Some(b1);
	self_.base.body_b = Some(b2);
	let d: B2vec2 = anchor2 - anchor1;
	self_.length = b2_max(d.length(), B2_LINEAR_SLOP);
	self_.min_length = self_.length;
	self_.max_length = self_.length;
}

pub(crate) fn b2_distance_joint_new<D: UserDataType>(
	def: &B2distanceJointDef<D>,
) -> B2distanceJoint<D> {
	let m_min_length = b2_max(def.min_length, B2_LINEAR_SLOP);
	return B2distanceJoint {
		base: B2joint::new(&def.base),
		m_local_anchor_a: def.local_anchor_a,
		m_local_anchor_b: def.local_anchor_b,
		m_length: b2_max(def.length, B2_LINEAR_SLOP),
		m_min_length: m_min_length,
		m_max_length: b2_max(def.max_length, m_min_length),
		m_stiffness: def.stiffness,
		m_damping: def.damping,
		m_gamma: 0.0,
		m_bias: 0.0,
		m_impulse: 0.0,
		m_lower_impulse: 0.0,
		m_upper_impulse: 0.0,
		m_current_length: 0.0,

		m_index_a: 0,
		m_index_b: 0,
		m_u: B2vec2::default(),
		m_r_a: B2vec2::default(),
		m_r_b: B2vec2::default(),
		m_local_center_a: B2vec2::default(),
		m_local_center_b: B2vec2::default(),
		m_inv_mass_a: 0.0,
		m_inv_mass_b: 0.0,
		m_inv_ia: 0.0,
		m_inv_ib: 0.0,
		m_mass: 0.0,
		m_soft_mass: 0.0,
	};
}

pub(crate) fn init_velocity_constraints<D: UserDataType>(
	self_: &mut B2distanceJoint<D>,
	data: &B2solverData,
	positions: &[B2position],
	velocities: &mut [B2velocity],
) {
	let m_body_a = self_.base.m_body_a.borrow();
	let m_body_b = self_.base.m_body_b.borrow();

	self_.m_index_a = m_body_a.m_island_index as usize;
	self_.m_index_b = m_body_b.m_island_index as usize;
	self_.m_local_center_a = m_body_a.m_sweep.local_center;
	self_.m_local_center_b = m_body_b.m_sweep.local_center;
	self_.m_inv_mass_a = m_body_a.m_inv_mass;
	self_.m_inv_mass_b = m_body_b.m_inv_mass;
	self_.m_inv_ia = m_body_a.m_inv_i;
	self_.m_inv_ib = m_body_b.m_inv_i;

	let c_a: B2vec2 = positions[self_.m_index_a].c;
	let a_a: f32 = positions[self_.m_index_a].a;
	let mut v_a: B2vec2 = velocities[self_.m_index_a].v;
	let mut w_a: f32 = velocities[self_.m_index_a].w;

	let c_b: B2vec2 = positions[self_.m_index_b].c;
	let a_b: f32 = positions[self_.m_index_b].a;
	let mut v_b: B2vec2 = velocities[self_.m_index_b].v;
	let mut w_b: f32 = velocities[self_.m_index_b].w;

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	self_.m_r_a = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
	self_.m_r_b = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);
	self_.m_u = c_b + self_.m_r_b - c_a - self_.m_r_a;

	// Handle singularity.
	self_.m_current_length = self_.m_u.length();
	if self_.m_current_length > B2_LINEAR_SLOP {
		self_.m_u *= 1.0 / self_.m_current_length;
	} else {
		self_.m_u.set(0.0, 0.0);
		self_.m_mass = 0.0;
		self_.m_impulse = 0.0;
		self_.m_lower_impulse = 0.0;
		self_.m_upper_impulse = 0.0;
	}

	let cr_au: f32 = b2_cross(self_.m_r_a, self_.m_u);
	let cr_bu: f32 = b2_cross(self_.m_r_b, self_.m_u);
	let mut inv_mass: f32 = self_.m_inv_mass_a
		+ self_.m_inv_ia * cr_au * cr_au
		+ self_.m_inv_mass_b
		+ self_.m_inv_ib * cr_bu * cr_bu;

	self_.m_mass = if inv_mass != 0.0 { 1.0 / inv_mass } else { 0.0 };

	if self_.m_stiffness > 0.0 && self_.m_min_length < self_.m_max_length {
		// soft
		let c: f32 = self_.m_current_length - self_.m_length;

		let d: f32 = self_.m_damping;
		let k: f32 = self_.m_stiffness;

		// magic formulas
		let h: f32 = data.step.dt;

		// gamma = 1 / (h * (d + h * k))
		// the extra factor of h in the denominator is since the lambda is an impulse, not a force
		self_.m_gamma = h * (d + h * k);
		self_.m_gamma = if self_.m_gamma != 0.0 {
			1.0 / self_.m_gamma
		} else {
			0.0
		};
		self_.m_bias = c * h * k * self_.m_gamma;

		inv_mass += self_.m_gamma;
		self_.m_soft_mass = if inv_mass != 0.0 { 1.0 / inv_mass } else { 0.0 };
	} else {
		// rigid
		self_.m_gamma = 0.0;
		self_.m_bias = 0.0;
		self_.m_soft_mass = self_.m_mass;
	}

	if data.step.warm_starting {
		// Scale the impulse to support a variable time step.
		self_.m_impulse *= data.step.dt_ratio;
		self_.m_lower_impulse *= data.step.dt_ratio;
		self_.m_upper_impulse *= data.step.dt_ratio;

		let p: B2vec2 = (self_.m_impulse + self_.m_lower_impulse - self_.m_upper_impulse) * self_.m_u;
		v_a -= self_.m_inv_mass_a * p;
		w_a -= self_.m_inv_ia * b2_cross(self_.m_r_a, p);
		v_b += self_.m_inv_mass_b * p;
		w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, p);
	} else {
		self_.m_impulse = 0.0;
	}

	velocities[self_.m_index_a].v = v_a;
	velocities[self_.m_index_a].w = w_a;
	velocities[self_.m_index_b].v = v_b;
	velocities[self_.m_index_b].w = w_b;
}

pub(crate) fn solve_velocity_constraints<D: UserDataType>(
	self_: &mut B2distanceJoint<D>,
	data: &B2solverData,
	velocities: &mut [B2velocity],
) {
	let mut v_a: B2vec2 = velocities[self_.m_index_a].v;
	let mut w_a: f32 = velocities[self_.m_index_a].w;
	let mut v_b: B2vec2 = velocities[self_.m_index_b].v;
	let mut w_b: f32 = velocities[self_.m_index_b].w;

	if self_.m_min_length < self_.m_max_length {
		if self_.m_stiffness > 0.0 {
			// Cdot = dot(u, v + cross(w, r))
			let vp_a: B2vec2 = v_a + b2_cross_scalar_by_vec(w_a, self_.m_r_a);
			let vp_b: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b);
			let cdot: f32 = b2_dot(self_.m_u, vp_b - vp_a);

			let impulse: f32 =
				-self_.m_soft_mass * (cdot + self_.m_bias + self_.m_gamma * self_.m_impulse);
			self_.m_impulse += impulse;

			let p: B2vec2 = impulse * self_.m_u;
			v_a -= self_.m_inv_mass_a * p;
			w_a -= self_.m_inv_ia * b2_cross(self_.m_r_a, p);
			v_b += self_.m_inv_mass_b * p;
			w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, p);
		}

		// lower
		{
			let c: f32 = self_.m_current_length - self_.m_min_length;
			let bias: f32 = b2_max(0.0, c) * data.step.inv_dt;

			let vp_a: B2vec2 = v_a + b2_cross_scalar_by_vec(w_a, self_.m_r_a);
			let vp_b: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b);
			let cdot: f32 = b2_dot(self_.m_u, vp_b - vp_a);

			let mut impulse: f32 = -self_.m_mass * (cdot + bias);
			let old_impulse: f32 = self_.m_lower_impulse;
			self_.m_lower_impulse = b2_max(0.0, self_.m_lower_impulse + impulse);
			impulse = self_.m_lower_impulse - old_impulse;
			let p: B2vec2 = impulse * self_.m_u;

			v_a -= self_.m_inv_mass_a * p;
			w_a -= self_.m_inv_ia * b2_cross(self_.m_r_a, p);
			v_b += self_.m_inv_mass_b * p;
			w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, p);
		}

		// upper
		{
			let c = self_.m_max_length - self_.m_current_length;
			let bias = b2_max(0.0, c) * data.step.inv_dt;

			let vp_a: B2vec2 = v_a + b2_cross_scalar_by_vec(w_a, self_.m_r_a);
			let vp_b: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b);
			let cdot: f32 = b2_dot(self_.m_u, vp_a - vp_b);

			let mut impulse: f32 = -self_.m_mass * (cdot + bias);
			let old_impulse: f32 = self_.m_upper_impulse;
			self_.m_upper_impulse = b2_max(0.0, self_.m_upper_impulse + impulse);
			impulse = self_.m_upper_impulse - old_impulse;
			let p: B2vec2 = -impulse * self_.m_u;

			v_a -= self_.m_inv_mass_a * p;
			w_a -= self_.m_inv_ia * b2_cross(self_.m_r_a, p);
			v_b += self_.m_inv_mass_b * p;
			w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, p);
		}
	} else {
		// Equal limits

		// Cdot = dot(u, v + cross(w, r))
		let vp_a: B2vec2 = v_a + b2_cross_scalar_by_vec(w_a, self_.m_r_a);
		let vp_b: B2vec2 = v_b + b2_cross_scalar_by_vec(w_b, self_.m_r_b);
		let cdot: f32 = b2_dot(self_.m_u, vp_b - vp_a);

		let impulse: f32 = -self_.m_mass * cdot;
		self_.m_impulse += impulse;

		let p: B2vec2 = impulse * self_.m_u;
		v_a -= self_.m_inv_mass_a * p;
		w_a -= self_.m_inv_ia * b2_cross(self_.m_r_a, p);
		v_b += self_.m_inv_mass_b * p;
		w_b += self_.m_inv_ib * b2_cross(self_.m_r_b, p);
	}

	velocities[self_.m_index_a].v = v_a;
	velocities[self_.m_index_a].w = w_a;
	velocities[self_.m_index_b].v = v_b;
	velocities[self_.m_index_b].w = w_b;
}

pub(crate) fn solve_position_constraints<D: UserDataType>(
	self_: &mut B2distanceJoint<D>,
	_data: &B2solverData,
	positions: &mut [B2position],
) -> bool {
	
	let mut c_a: B2vec2 = positions[self_.m_index_a].c;
	let mut a_a: f32 = positions[self_.m_index_a].a;
	let mut c_b: B2vec2 = positions[self_.m_index_b].c;
	let mut a_b: f32 = positions[self_.m_index_b].a;

	let (q_a, q_b) = (B2Rot::new(a_a), B2Rot::new(a_b));

	let r_a: B2vec2 = b2_mul_rot_by_vec2(q_a, self_.m_local_anchor_a - self_.m_local_center_a);
	let r_b: B2vec2 = b2_mul_rot_by_vec2(q_b, self_.m_local_anchor_b - self_.m_local_center_b);
	let mut u: B2vec2 = c_b + r_b - c_a - r_a;

	let length: f32 = u.normalize();
	let c: f32;
	if self_.m_min_length == self_.m_max_length {
		c = length - self_.m_min_length;
	} else if length < self_.m_min_length {
		c = length - self_.m_min_length;
	} else if self_.m_max_length < length {
		c = length - self_.m_max_length;
	} else {
		return true;
	}

	let impulse: f32 = -self_.m_mass * c;
	let p: B2vec2 = impulse * u;

	c_a -= self_.m_inv_mass_a * p;
	a_a -= self_.m_inv_ia * b2_cross(r_a, p);
	c_b += self_.m_inv_mass_b * p;
	a_b += self_.m_inv_ib * b2_cross(r_b, p);

	positions[self_.m_index_a].c = c_a;
	positions[self_.m_index_a].a = a_a;
	positions[self_.m_index_b].c = c_b;
	positions[self_.m_index_b].a = a_b;

	return b2_abs(c) < B2_LINEAR_SLOP;
}

pub(crate) fn get_anchor_a<D: UserDataType>(self_: &B2distanceJoint<D>) -> B2vec2 {
	return self_
		.base
		.m_body_a
		.borrow()
		.get_world_point(self_.m_local_anchor_a);
}

pub(crate) fn get_anchor_b<D: UserDataType>(self_: &B2distanceJoint<D>) -> B2vec2 {
	return self_
		.base
		.m_body_b
		.borrow()
		.get_world_point(self_.m_local_anchor_b);
}

pub(crate) fn get_reaction_force<D: UserDataType>(
	self_: &B2distanceJoint<D>,
	inv_dt: f32,
) -> B2vec2 {
	let f: B2vec2 =
		inv_dt * (self_.m_impulse + self_.m_lower_impulse - self_.m_upper_impulse) * self_.m_u;
	return f;
}

pub(crate) fn get_reaction_torque<D: UserDataType>(_self: &B2distanceJoint<D>, inv_dt: f32) -> f32 {
	b2_not_used(inv_dt);
	return 0.0;
}

pub(crate) fn set_length<D: UserDataType>(self_: &mut B2distanceJoint<D>, length: f32) -> f32 {
	self_.m_impulse = 0.0;
	self_.m_length = b2_max(B2_LINEAR_SLOP, length);
	return self_.m_length;
}

pub(crate) fn set_min_length<D: UserDataType>(self_: &mut B2distanceJoint<D>, min_length: f32) -> f32 {
	self_.m_lower_impulse = 0.0;
	self_.m_min_length = b2_clamp(min_length, B2_LINEAR_SLOP, self_.m_max_length);
	return self_.m_min_length;
}

pub(crate) fn set_max_length<D: UserDataType>(self_: &mut B2distanceJoint<D>, max_length: f32) -> f32 {
	self_.m_upper_impulse = 0.0;
	self_.m_max_length = b2_max(max_length, self_.m_min_length);
	return self_.m_max_length;
}

pub(crate) fn get_current_length<D: UserDataType>(self_: &B2distanceJoint<D>) -> f32 {
	let p_a: B2vec2 = self_
		.base
		.m_body_a
		.borrow()
		.get_world_point(self_.m_local_anchor_a);
	let p_b: B2vec2 = self_
		.base
		.m_body_b
		.borrow()
		.get_world_point(self_.m_local_anchor_b);
	let d: B2vec2 = p_b - p_a;
	let length: f32 = d.length();
	return length;
}

pub(crate) fn draw<D: UserDataType>(self_: &B2distanceJoint<D>, draw: &mut dyn B2drawTrait) {
	let xf_a = self_.base.m_body_a.borrow().get_transform();
	let xf_b = self_.base.m_body_b.borrow().get_transform();
	let p_a: B2vec2 = b2_mul_transform_by_vec2(xf_a, self_.m_local_anchor_a);
	let p_b: B2vec2 = b2_mul_transform_by_vec2(xf_b, self_.m_local_anchor_b);

	let mut axis: B2vec2 = p_b - p_a;
	let _length: f32 = axis.normalize();

	let c1 = B2color::new(0.7, 0.7, 0.7);
	let c2 = B2color::new(0.3, 0.9, 0.3);
	let c3 = B2color::new(0.9, 0.3, 0.3);
	let c4 = B2color::new(0.4, 0.4, 0.4);

	draw.draw_segment(p_a, p_b, c4);
	let p_rest: B2vec2 = p_a + self_.m_length * axis;
	draw.draw_point(p_rest, 8.0, c1);

	if self_.m_min_length != self_.m_max_length {
		if self_.m_min_length > B2_LINEAR_SLOP {
			let p_min: B2vec2 = p_a + self_.m_min_length * axis;
			draw.draw_point(p_min, 4.0, c2);
		}

		if self_.m_max_length < B2_MAX_FLOAT {
			let p_max: B2vec2 = p_a + self_.m_max_length * axis;
			draw.draw_point(p_max, 4.0, c3);
		}
	}
}
