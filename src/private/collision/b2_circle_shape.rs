use crate::shapes::b2_circle_shape::*;
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;

pub fn clone(self_: &B2circleShape) -> Box<dyn B2shapeDynTrait> {
	return Box::new(B2circleShape::clone(&self_));
}

pub fn get_child_count(_self: &B2circleShape) -> usize {
	return 1;
}

pub fn test_point(self_: &B2circleShape, transform: B2Transform, p: B2vec2) -> bool {
	let center: B2vec2 = transform.p + b2_mul_rot_by_vec2(transform.q, self_.m_p);
	let d: B2vec2 = p - center;
	return b2_dot(d, d) <= self_.base.m_radius * self_.base.m_radius;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius

pub fn ray_cast(
	self_: &B2circleShape,
	output: &mut B2rayCastOutput,
	input: &B2rayCastInput,
	transform: B2Transform,
	child_index: usize,
) -> bool {
	b2_not_used(child_index);

	let position: B2vec2 = transform.p + b2_mul_rot_by_vec2(transform.q, self_.m_p);
	let s: B2vec2 = input.p1 - position;
	let b: f32 = b2_dot(s, s) - self_.base.m_radius * self_.base.m_radius;

	// solve quadratic equation.
	let r: B2vec2 = input.p2 - input.p1;
	let c: f32 = b2_dot(s, r);
	let rr: f32 = b2_dot(r, r);
	let sigma: f32 = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if sigma < 0.0 || rr < B2_EPSILON {
		return false;
	}

	// Find the point of intersection of the line with the circle.
	let mut a: f32 = -(c + b2_sqrt(sigma));

	// Is the intersection point on the segment?
	if 0.0 <= a && a <= input.max_fraction * rr {
		a /= rr;
		output.fraction = a;
		output.normal = s + a * r;
		output.normal.normalize();
		return true;
	}

	return false;
}
pub fn compute_aabb(
	self_: &B2circleShape,
	aabb: &mut B2AABB,
	transform: B2Transform,
	child_index: usize,
) {
	b2_not_used(child_index);

	let p: B2vec2 = transform.p + b2_mul_rot_by_vec2(transform.q, self_.m_p);
	aabb.lower_bound
		.set(p.x - self_.base.m_radius, p.y - self_.base.m_radius);
	aabb.upper_bound
		.set(p.x + self_.base.m_radius, p.y + self_.base.m_radius);
}

pub fn compute_mass(self_: &B2circleShape, mass_data: &mut B2massData, density: f32) {
	mass_data.mass = density * B2_PI * self_.base.m_radius * self_.base.m_radius;
	mass_data.center = self_.m_p;

	// inertia about the local origin
	mass_data.i = mass_data.mass
		* (0.5 * self_.base.m_radius * self_.base.m_radius + b2_dot(self_.m_p, self_.m_p));
}
