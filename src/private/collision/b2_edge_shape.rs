use crate::b2_collision::*;
use crate::shapes::b2_edge_shape::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;

pub fn b2_set_one_sided(self_: &mut B2edgeShape, v0: B2vec2, v1: B2vec2, v2: B2vec2, v3: B2vec2) {
	self_.m_vertex0 = v0;
	self_.m_vertex1 = v1;
	self_.m_vertex2 = v2;
	self_.m_vertex3 = v3;
	self_.m_one_sided = true;
}

pub fn b2_set_two_sided(self_: &mut B2edgeShape, v1: B2vec2, v2: B2vec2) {
	self_.m_vertex1 = v1;
	self_.m_vertex2 = v2;
	self_.m_one_sided = false;
}

pub fn b2_shape_dyn_trait_clone(self_: &B2edgeShape) -> Box<dyn B2shapeDynTrait> {
	return Box::new(B2edgeShape::clone(&self_));
}

pub fn b2_shape_dyn_trait_get_child_count(_self: &B2edgeShape) -> usize {
	return 1;
}

pub fn b2_shape_dyn_trait_test_point(_self: &B2edgeShape, xf: B2Transform, p: B2vec2) -> bool {
	b2_not_used(xf);
	b2_not_used(p);
	return false;
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
pub fn b2_shape_dyn_trait_ray_cast(
	self_: &B2edgeShape,
	output: &mut B2rayCastOutput,
	input: &B2rayCastInput,
	xf: B2Transform,
	child_index: usize,
) -> bool {
	b2_not_used(child_index);

	// Put the ray into the edge's frame of reference.
	let p1: B2vec2 = b2_mul_t_rot_by_vec2(xf.q, input.p1 - xf.p);
	let p2: B2vec2 = b2_mul_t_rot_by_vec2(xf.q, input.p2 - xf.p);
	let d: B2vec2 = p2 - p1;

	let v1: B2vec2 = self_.m_vertex1;
	let v2: B2vec2 = self_.m_vertex2;
	let e: B2vec2 = v2 - v1;
	// Normal points to the right, looking from v1 at v2
	let mut normal = B2vec2 { x: e.y, y: -e.x };
	normal.normalize();

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	let numerator: f32 = b2_dot(normal, v1 - p1);
	
	if self_.m_one_sided && numerator > 0.0 {
		return false;
	}

	let denominator: f32 = b2_dot(normal, d);
	if denominator == 0.0
	{
		return false;
	}
	let t: f32 = numerator / denominator;
	if t < 0.0 || input.max_fraction < t
	{
		return false;
	}

	let q: B2vec2 = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	let r: B2vec2 = v2 - v1;
	let rr: f32 = b2_dot(r, r);
	if rr == 0.0 {
		return false;
	}

	let s: f32 = b2_dot(q - v1, r) / rr;
	if s < 0.0 || 1.0 < s {
		return false;
	}

	output.fraction = t;
	if numerator > 0.0 {
		output.normal = -b2_mul_rot_by_vec2(xf.q, normal);
	} else {
		output.normal = b2_mul_rot_by_vec2(xf.q, normal);
	}
	return true;
}

pub fn b2_shape_dyn_trait_compute_aabb(
	self_: &B2edgeShape,
	aabb: &mut B2AABB,
	xf: B2Transform,
	child_index: usize,
) {
	b2_not_used(child_index);

	let v1: B2vec2 = b2_mul_transform_by_vec2(xf, self_.m_vertex1);
	let v2: B2vec2 = b2_mul_transform_by_vec2(xf, self_.m_vertex2);

	let lower: B2vec2 = b2_min_vec2(v1, v2);
	let upper: B2vec2 = b2_max_vec2(v1, v2);

	let r = B2vec2 {
		x: self_.base.m_radius,
		y: self_.base.m_radius,
	};
	aabb.lower_bound = lower - r;
	aabb.upper_bound = upper + r;
}

pub fn b2_shape_dyn_trait_compute_mass(self_: &B2edgeShape, mass_data: &mut B2massData, density: f32) {
	b2_not_used(density);

	mass_data.mass = 0.0;
	mass_data.center = 0.5 * (self_.m_vertex1 + self_.m_vertex2);
	mass_data.i = 0.0;
}
