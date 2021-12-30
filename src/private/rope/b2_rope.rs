use crate::b2_rope::*;
use crate::b2_draw::*;
use crate::b2_math::*;
use crate::b2_common::*;

pub(crate) fn default() -> B2rope {
	B2rope {
		m_position: B2vec2::zero(),
		m_positions: Vec::default(),
		m_stretch_count: 0,
		m_bend_count: 0,
		m_gravity: B2vec2::zero(),
		m_stretch_constraints: Vec::default(),
		m_bend_constraints: Vec::default(),
		m_tuning: Default::default(),
	}
}

pub(crate) fn create(_self: &mut B2rope, def: &B2ropeDef) {
	b2_assert(def.vertices.len() >= 3);
	_self.m_position = def.position;

	let m_count = def.vertices.len();

	_self.m_positions.resize(m_count, Default::default());

	for (i,def) in def.vertices.iter().enumerate() {
		let mut new_pos = B2ropePositions::default();
		new_pos.m_bind_positions = def.position;
		new_pos.m_ps = def.position + _self.m_position;
		new_pos.m_p0s = def.position + _self.m_position;
		new_pos.m_vs.set_zero();

		let m: f32 = def.mass;
		if m > 0.0 {
			new_pos.m_inv_masses = 1.0 / m;
		} else {
			new_pos.m_inv_masses = 0.0;
		}

		_self.m_positions[i]=new_pos;
	}

	_self.m_stretch_count = m_count - 1;
	_self.m_bend_count = m_count - 2;

	_self
		.m_stretch_constraints
		.resize(_self.m_stretch_count, Default::default());
	_self
		.m_bend_constraints
		.resize(_self.m_bend_count, Default::default());

	for i in 0.._self.m_stretch_count {
		let c: &mut B2ropeStretch = &mut _self.m_stretch_constraints[i];

		let p1: B2vec2 = _self.m_positions[i].m_ps;
		let p2: B2vec2 = _self.m_positions[i + 1].m_ps;

		c.i1 = i as i32;
		c.i2 = (i + 1) as i32;
		c.l = b2_distance_vec2(p1, p2);
		c.inv_mass1 = _self.m_positions[i].m_inv_masses;
		c.inv_mass2 = _self.m_positions[i + 1].m_inv_masses;
		c.lambda = 0.0;
		c.damper = 0.0;
		c.spring = 0.0;
	}

	for i in 0.._self.m_bend_count {
		let c: &mut B2ropeBend = &mut _self.m_bend_constraints[i];

		let p1: B2vec2 = _self.m_positions[i].m_ps;
		let p2: B2vec2 = _self.m_positions[i + 1].m_ps;
		let p3: B2vec2 = _self.m_positions[i + 2].m_ps;

		c.i1 = i as i32;
		c.i2 = (i + 1) as i32;
		c.i3 = (i + 2) as i32;
		c.inv_mass1 = _self.m_positions[i].m_inv_masses;
		c.inv_mass2 = _self.m_positions[i + 1].m_inv_masses;
		c.inv_mass3 = _self.m_positions[i + 2].m_inv_masses;
		c.inv_effective_mass = 0.0;
		c.l1 = b2_distance_vec2(p1, p2);
		c.l2 = b2_distance_vec2(p2, p3);
		c.lambda = 0.0;

		// Pre-compute effective mass (TODO use flattened config)
		let e1: B2vec2 = p2 - p1;
		let e2: B2vec2 = p3 - p2;
		let l1sqr: f32 = e1.length_squared();
		let l2sqr: f32 = e2.length_squared();

		if l1sqr * l2sqr == 0.0 {
			continue;
		}

		let jd1: B2vec2 = (-1.0 / l1sqr) * e1.skew();
		let jd2: B2vec2 = (1.0 / l2sqr) * e2.skew();

		let j1: B2vec2 = -jd1;
		let j2: B2vec2 = jd1 - jd2;
		let j3: B2vec2 = jd2;

		c.inv_effective_mass =
			c.inv_mass1 * b2_dot(j1, j1) + c.inv_mass2 * b2_dot(j2, j2) + c.inv_mass3 * b2_dot(j3, j3);

		let r: B2vec2 = p3 - p1;

		let rr: f32 = r.length_squared();
		if rr == 0.0 {
			continue;
		}

		// a1 = h2 / (h1 + h2)
		// a2 = h1 / (h1 + h2)
		c.alpha1 = b2_dot(e2, r) / rr;
		c.alpha2 = b2_dot(e1, r) / rr;
	}

	_self.m_gravity = def.gravity;

	set_tuning(_self, &def.tuning);
}

pub(crate) fn set_tuning(_self: &mut B2rope, tuning: &B2ropeTuning) {
	_self.m_tuning = *tuning;

	// Pre-compute spring and damper values based on tuning

	let bend_omega: f32 = 2.0 * B2_PI * _self.m_tuning.bend_hertz;

	for c in &mut _self.m_bend_constraints {
		let l1sqr: f32 = c.l1 * c.l1;
		let l2sqr: f32 = c.l2 * c.l2;

		if l1sqr * l2sqr == 0.0 {
			c.spring = 0.0;
			c.damper = 0.0;
			continue;
		}

		// Flatten the triangle formed by the two edges
		let j2: f32 = 1.0 / c.l1 + 1.0 / c.l2;
		let sum: f32 = c.inv_mass1 / l1sqr + c.inv_mass2 * j2 * j2 + c.inv_mass3 / l2sqr;
		if sum == 0.0 {
			c.spring = 0.0;
			c.damper = 0.0;
			continue;
		}

		let mass: f32 = 1.0 / sum;

		c.spring = mass * bend_omega * bend_omega;
		c.damper = 2.0 * mass * _self.m_tuning.bend_damping * bend_omega;
	}
	let stretch_omega: f32 = 2.0 * B2_PI * _self.m_tuning.stretch_hertz;

	for c in &mut _self.m_stretch_constraints {
		let sum: f32 = c.inv_mass1 + c.inv_mass2;
		if sum == 0.0 {
			continue;
		}

		let mass: f32 = 1.0 / sum;

		c.spring = mass * stretch_omega * stretch_omega;
		c.damper = 2.0 * mass * _self.m_tuning.stretch_damping * stretch_omega;
	}
}

pub(crate) fn step(_self: &mut B2rope, dt: f32, iterations: i32, position: B2vec2) {
	if dt == 0.0 {
		return;
	}

	let inv_dt: f32 = 1.0 / dt;
	let d: f32 = f32::exp(-dt * _self.m_tuning.damping);

	// Apply gravity and damping
	for p in &mut _self.m_positions {
		if p.m_inv_masses > 0.0 {
			p.m_vs *= d;
			p.m_vs += dt * _self.m_gravity;
		} else {
			p.m_vs = inv_dt * (p.m_bind_positions + position - p.m_p0s);
		}
	}

	// Apply bending spring
	if _self.m_tuning.bending_model == B2bendingModel::B2SpringAngleBendingModel {
		apply_bend_forces(_self, dt);
	}

	for c in &mut _self.m_bend_constraints {
		c.lambda = 0.0;
	}

	for c in &mut _self.m_stretch_constraints {
		c.lambda = 0.0;
	}

	// update position
	for p in &mut _self.m_positions {
		p.m_ps += dt * p.m_vs;
	}

	// solve constraints
	for _i in 0..iterations {
		match _self.m_tuning.bending_model {
			B2bendingModel::B2SpringAngleBendingModel => {
				//nothing
			}
			B2bendingModel::B2PbdAngleBendingModel => {
				solve_bend_pbd_angle(_self);
			}
			B2bendingModel::B2XpbdAngleBendingModel => {
				solve_bend_xpbd_angle(_self, dt);
			}
			B2bendingModel::B2PbdDistanceBendingModel => {
				solve_bend_pbd_distance(_self);
			}
			B2bendingModel::B2PbdHeightBendingModel => {
				solve_bend_pbd_height(_self);
			}
			B2bendingModel::B2PbdTriangleBendingModel => {
				solve_bend_pbd_triangle(_self);
			}
		}

		match _self.m_tuning.stretching_model {
			B2stretchingModel::B2PbdStretchingModel => {
				solve_stretch_pbd(_self);
			}
			B2stretchingModel::B2XpbdStretchingModel => {
				solve_stretch_xpbd(_self, dt);
			}
		}
	}

	// Constrain velocity
	for p in &mut _self.m_positions {
		p.m_vs = inv_dt * (p.m_ps - p.m_p0s);
		p.m_p0s = p.m_ps;
	}
}

pub(crate) fn reset(_self: &mut B2rope, position: B2vec2) {
	_self.m_position = position;

	for p in &mut _self.m_positions {
		p.m_ps = p.m_bind_positions + _self.m_position;
		p.m_p0s = p.m_bind_positions + _self.m_position;
		p.m_vs.set_zero();
	}

	for c in &mut _self.m_bend_constraints {
		c.lambda = 0.0;
	}

	for c in &mut _self.m_stretch_constraints {
		c.lambda = 0.0;
	}
}

pub(crate) fn solve_stretch_pbd(_self: &mut B2rope) {
	let stiffness: f32 = _self.m_tuning.stretch_stiffness;

	for c in &_self.m_stretch_constraints {
		let mut p1: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let mut p2: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;

		let mut d: B2vec2 = p2 - p1;
		let l: f32 = d.normalize();

		let sum: f32 = c.inv_mass1 + c.inv_mass2;
		if sum == 0.0 {
			continue;
		}

		let s1: f32 = c.inv_mass1 / sum;
		let s2: f32 = c.inv_mass2 / sum;

		p1 -= stiffness * s1 * (c.l - l) * d;
		p2 += stiffness * s2 * (c.l - l) * d;

		_self.m_positions[c.i1 as usize].m_ps = p1;
		_self.m_positions[c.i2 as usize].m_ps = p2;
	}
}

pub(crate) fn solve_stretch_xpbd(_self: &mut B2rope, dt: f32) {
	b2_assert(dt > 0.0);

	for c in &mut _self.m_stretch_constraints {
		let mut p1: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let mut p2: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;

		let dp1: B2vec2 = p1 - _self.m_positions[c.i1 as usize].m_p0s;
		let dp2: B2vec2 = p2 - _self.m_positions[c.i2 as usize].m_p0s;

		let mut u: B2vec2 = p2 - p1;
		let l: f32 = u.normalize();

		let j1: B2vec2 = -u;
		let j2: B2vec2 = u;

		let sum: f32 = c.inv_mass1 + c.inv_mass2;
		if sum == 0.0 {
			continue;
		}

		let alpha: f32 = 1.0 / (c.spring * dt * dt); // 1 / kg
		let beta: f32 = dt * dt * c.damper; // kg * s
		let sigma: f32 = alpha * beta / dt; // non-dimensional
		let c1: f32 = l - c.l;

		// This is using the initial velocities
		let cdot: f32 = b2_dot(j1, dp1) + b2_dot(j2, dp2);

		let b: f32 = c1 + alpha * c.lambda + sigma * cdot;
		let sum2: f32 = (1.0 + sigma) * sum + alpha;

		let impulse: f32 = -b / sum2;

		p1 += (c.inv_mass1 * impulse) * j1;
		p2 += (c.inv_mass2 * impulse) * j2;

		_self.m_positions[c.i1 as usize].m_ps = p1;
		_self.m_positions[c.i2 as usize].m_ps = p2;
		c.lambda += impulse;
	}
}

pub(crate) fn solve_bend_pbd_angle(_self: &mut B2rope) {
	let stiffness: f32 = _self.m_tuning.bend_stiffness;

	for c in &_self.m_bend_constraints {
		let mut p1: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let mut p2: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;
		let mut p3: B2vec2 = _self.m_positions[c.i3 as usize].m_ps;

		let d1: B2vec2 = p2 - p1;
		let d2: B2vec2 = p3 - p2;
		let a: f32 = b2_cross(d1, d2);
		let b: f32 = b2_dot(d1, d2);

		let angle: f32 = b2_atan2(a, b);

		let l1sqr: f32;
		let l2sqr: f32;

		if _self.m_tuning.isometric {
			l1sqr = c.l1 * c.l1;
			l2sqr = c.l2 * c.l2;
		} else {
			l1sqr = d1.length_squared();
			l2sqr = d2.length_squared();
		}

		if l1sqr * l2sqr == 0.0 {
			continue;
		}

		let jd1: B2vec2 = (-1.0 / l1sqr) * d1.skew();
		let jd2: B2vec2 = (1.0 / l2sqr) * d2.skew();

		let j1: B2vec2 = -jd1;
		let j2: B2vec2 = jd1 - jd2;
		let j3: B2vec2 = jd2;

		let mut sum: f32;
		if _self.m_tuning.fixed_effective_mass {
			sum = c.inv_effective_mass;
		} else {
			sum = c.inv_mass1 * b2_dot(j1, j1)
				+ c.inv_mass2 * b2_dot(j2, j2)
				+ c.inv_mass3 * b2_dot(j3, j3);
		}

		if sum == 0.0 {
			sum = c.inv_effective_mass;
		}

		let impulse: f32 = -stiffness * angle / sum;

		p1 += (c.inv_mass1 * impulse) * j1;
		p2 += (c.inv_mass2 * impulse) * j2;
		p3 += (c.inv_mass3 * impulse) * j3;

		_self.m_positions[c.i1 as usize].m_ps = p1;
		_self.m_positions[c.i2 as usize].m_ps = p2;
		_self.m_positions[c.i3 as usize].m_ps = p3;
	}
}

pub(crate) fn solve_bend_xpbd_angle(_self: &mut B2rope, dt: f32) {
	b2_assert(dt > 0.0);

	for c in &mut _self.m_bend_constraints {
		let mut p1: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let mut p2: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;
		let mut p3: B2vec2 = _self.m_positions[c.i3 as usize].m_ps;

		let dp1: B2vec2 = p1 - _self.m_positions[c.i1 as usize].m_p0s;
		let dp2: B2vec2 = p2 - _self.m_positions[c.i2 as usize].m_p0s;
		let dp3: B2vec2 = p3 - _self.m_positions[c.i3 as usize].m_p0s;

		let d1: B2vec2 = p2 - p1;
		let d2: B2vec2 = p3 - p2;

		let l1sqr: f32;
		let l2sqr: f32;

		if _self.m_tuning.isometric {
			l1sqr = c.l1 * c.l1;
			l2sqr = c.l2 * c.l2;
		} else {
			l1sqr = d1.length_squared();
			l2sqr = d2.length_squared();
		}

		if l1sqr * l2sqr == 0.0 {
			continue;
		}

		let a: f32 = b2_cross(d1, d2);
		let b: f32 = b2_dot(d1, d2);

		let angle: f32 = b2_atan2(a, b);

		let jd1: B2vec2 = (-1.0 / l1sqr) * d1.skew();
		let jd2: B2vec2 = (1.0 / l2sqr) * d2.skew();

		let j1: B2vec2 = -jd1;
		let j2: B2vec2 = jd1 - jd2;
		let j3: B2vec2 = jd2;

		let sum: f32;
		if _self.m_tuning.fixed_effective_mass {
			sum = c.inv_effective_mass;
		} else {
			sum = c.inv_mass1 * b2_dot(j1, j1)
				+ c.inv_mass2 * b2_dot(j2, j2)
				+ c.inv_mass3 * b2_dot(j3, j3);
		}

		if sum == 0.0 {
			continue;
		}

		let alpha: f32 = 1.0 / (c.spring * dt * dt);
		let beta: f32 = dt * dt * c.damper;
		let sigma: f32 = alpha * beta / dt;
		let c1: f32 = angle;

		// This is using the initial velocities
		let cdot: f32 = b2_dot(j1, dp1) + b2_dot(j2, dp2) + b2_dot(j3, dp3);

		let b: f32 = c1 + alpha * c.lambda + sigma * cdot;
		let sum2: f32 = (1.0 + sigma) * sum + alpha;

		let impulse: f32 = -b / sum2;

		p1 += (c.inv_mass1 * impulse) * j1;
		p2 += (c.inv_mass2 * impulse) * j2;
		p3 += (c.inv_mass3 * impulse) * j3;

		_self.m_positions[c.i1 as usize].m_ps = p1;
		_self.m_positions[c.i2 as usize].m_ps = p2;
		_self.m_positions[c.i3 as usize].m_ps = p3;
		c.lambda += impulse;
	}
}

pub(crate) fn apply_bend_forces(_self: &mut B2rope, dt: f32) {
	// omega = 2 * pi * hz
	let omega: f32 = 2.0 * B2_PI * _self.m_tuning.bend_hertz;

	for c in &_self.m_bend_constraints {
		let p1: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let p2: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;
		let p3: B2vec2 = _self.m_positions[c.i3 as usize].m_ps;

		let v1: B2vec2 = _self.m_positions[c.i1 as usize].m_vs;
		let v2: B2vec2 = _self.m_positions[c.i2 as usize].m_vs;
		let v3: B2vec2 = _self.m_positions[c.i3 as usize].m_vs;

		let d1: B2vec2 = p2 - p1;
		let d2: B2vec2 = p3 - p2;

		let l1sqr: f32;
		let l2sqr: f32;

		if _self.m_tuning.isometric {
			l1sqr = c.l1 * c.l1;
			l2sqr = c.l2 * c.l2;
		} else {
			l1sqr = d1.length_squared();
			l2sqr = d2.length_squared();
		}

		if l1sqr * l2sqr == 0.0 {
			continue;
		}

		let a: f32 = b2_cross(d1, d2);
		let b: f32 = b2_dot(d1, d2);

		let angle: f32 = b2_atan2(a, b);

		let jd1: B2vec2 = (-1.0 / l1sqr) * d1.skew();
		let jd2: B2vec2 = (1.0 / l2sqr) * d2.skew();

		let j1: B2vec2 = -jd1;
		let j2: B2vec2 = jd1 - jd2;
		let j3: B2vec2 = jd2;

		let sum: f32;
		if _self.m_tuning.fixed_effective_mass {
			sum = c.inv_effective_mass;
		} else {
			sum = c.inv_mass1 * b2_dot(j1, j1)
				+ c.inv_mass2 * b2_dot(j2, j2)
				+ c.inv_mass3 * b2_dot(j3, j3);
		}

		if sum == 0.0 {
			continue;
		}

		let mass: f32 = 1.0 / sum;

		let spring: f32 = mass * omega * omega;
		let damper: f32 = 2.0 * mass * _self.m_tuning.bend_damping * omega;

		let c1: f32 = angle;
		let cdot: f32 = b2_dot(j1, v1) + b2_dot(j2, v2) + b2_dot(j3, v3);

		let impulse: f32 = -dt * (spring * c1 + damper * cdot);

		_self.m_positions[c.i1 as usize].m_vs += (c.inv_mass1 * impulse) * j1;
		_self.m_positions[c.i2 as usize].m_vs += (c.inv_mass2 * impulse) * j2;
		_self.m_positions[c.i3 as usize].m_vs += (c.inv_mass3 * impulse) * j3;
	}
}

pub(crate) fn solve_bend_pbd_distance(_self: &mut B2rope) {
	let stiffness: f32 = _self.m_tuning.bend_stiffness;

	for c in &_self.m_bend_constraints {
		let i1: i32 = c.i1;
		let i2: i32 = c.i3;

		let mut p1: B2vec2 = _self.m_positions[i1 as usize].m_ps;
		let mut p2: B2vec2 = _self.m_positions[i2 as usize].m_ps;

		let mut d: B2vec2 = p2 - p1;
		let l: f32 = d.normalize();

		let sum: f32 = c.inv_mass1 + c.inv_mass3;
		if sum == 0.0 {
			continue;
		}

		let s1: f32 = c.inv_mass1 / sum;
		let s2: f32 = c.inv_mass3 / sum;

		p1 -= stiffness * s1 * (c.l1 + c.l2 - l) * d;
		p2 += stiffness * s2 * (c.l1 + c.l2 - l) * d;

		_self.m_positions[i1 as usize].m_ps = p1;
		_self.m_positions[i2 as usize].m_ps = p2;
	}
}

// Constraint based implementation of:
// p. Volino: Simple Linear Bending Stiffness in Particle Systems
pub(crate) fn solve_bend_pbd_height(_self: &mut B2rope) {
	let stiffness: f32 = _self.m_tuning.bend_stiffness;

	for c in &_self.m_bend_constraints {
		let mut p1: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let mut p2: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;
		let mut p3: B2vec2 = _self.m_positions[c.i3 as usize].m_ps;

		// Barycentric coordinates are held constant
		let d: B2vec2 = c.alpha1 * p1 + c.alpha2 * p3 - p2;
		let d_len: f32 = d.length();

		if d_len == 0.0 {
			continue;
		}

		let d_hat: B2vec2 = (1.0 / d_len) * d;

		let j1: B2vec2 = c.alpha1 * d_hat;
		let j2: B2vec2 = -d_hat;
		let j3: B2vec2 = c.alpha2 * d_hat;

		let sum: f32 =
			c.inv_mass1 * c.alpha1 * c.alpha1 + c.inv_mass2 + c.inv_mass3 * c.alpha2 * c.alpha2;

		if sum == 0.0 {
			continue;
		}

		let c1: f32 = d_len;
		let mass: f32 = 1.0 / sum;
		let impulse: f32 = -stiffness * mass * c1;

		p1 += (c.inv_mass1 * impulse) * j1;
		p2 += (c.inv_mass2 * impulse) * j2;
		p3 += (c.inv_mass3 * impulse) * j3;

		_self.m_positions[c.i1 as usize].m_ps = p1;
		_self.m_positions[c.i2 as usize].m_ps = p2;
		_self.m_positions[c.i3 as usize].m_ps = p3;
	}
}

// M. Kelager: A Triangle Bending Constraint Model for PBD
fn solve_bend_pbd_triangle(_self: &mut B2rope)
{
	let stiffness: f32 = _self.m_tuning.bend_stiffness;

	for c in &_self.m_bend_constraints
	{
		let mut b0: B2vec2 = _self.m_positions[c.i1 as usize].m_ps;
		let mut v: B2vec2 = _self.m_positions[c.i2 as usize].m_ps;
		let mut b1: B2vec2 = _self.m_positions[c.i3 as usize].m_ps;

		let wb0: f32 = c.inv_mass1;
		let wv: f32 = c.inv_mass2;
		let wb1: f32 = c.inv_mass3;

		let w: f32 = wb0 + wb1 + 2.0 * wv;
		let inv_w: f32 = stiffness / w;

		let d: B2vec2 = v - (1.0 / 3.0) * (b0 + v + b1);

		let db0: B2vec2 = 2.0 * wb0 * inv_w * d;
		let dv: B2vec2 = -4.0 * wv * inv_w * d;
		let db1: B2vec2 = 2.0 * wb1 * inv_w * d;

		b0 += db0;
		v += dv;
		b1 += db1;

		_self.m_positions[c.i1 as usize].m_ps = b0;
		_self.m_positions[c.i2 as usize].m_ps = v;
		_self.m_positions[c.i3 as usize].m_ps = b1;
	}
}

pub(crate) fn draw(_self: &B2rope, draw: &mut dyn B2drawTrait) {
	let c = B2color::new(0.4, 0.5, 0.7);
	let pg = B2color::new(0.1, 0.8, 0.1);
	let pd = B2color::new(0.7, 0.2, 0.4);

	let m_count = _self.m_positions.len();

	for i in 0..(m_count - 1) {
		draw.draw_segment(_self.m_positions[i].m_ps, _self.m_positions[i + 1].m_ps, c);

		let pc: B2color = if _self.m_positions[i].m_inv_masses > 0.0 {
			pd
		} else {
			pg
		};
		draw.draw_point(_self.m_positions[i].m_ps, 5.0, pc);
	}

	let pc: B2color = if _self.m_positions[m_count - 1].m_inv_masses > 0.0 {
		pd
	} else {
		pg
	};
	draw.draw_point(_self.m_positions[m_count - 1].m_ps, 5.0, pc);
}
