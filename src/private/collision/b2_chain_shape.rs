use crate::shapes::b2_chain_shape::*;
use crate::b2_collision::*;
use crate::shapes::b2_edge_shape::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;

pub fn b2_chain_shape_clear(this: &mut B2chainShape) {
	this.m_vertices.clear();
}

pub fn b2_chain_shape_create_loop(this: &mut B2chainShape, vertices: &[B2vec2]) {
	let count = vertices.len();
	b2_assert(this.m_vertices.len() == 0);
	b2_assert(count >= 3);
	if count < 3 {
		return;
	}

	for i in 1..count {
		let v1: B2vec2 = vertices[i - 1];
		let v2: B2vec2 = vertices[i];
		// If the code crashes here, it means your vertices are too close together.
		b2_assert(b2_distance_vec2_squared(v1, v2) > B2_LINEAR_SLOP * B2_LINEAR_SLOP);
	}

	this.m_vertices = Vec::from([vertices, &[vertices[0]]].concat());
	this.m_prev_vertex = this.m_vertices[this.m_vertices.len() - 2];
	this.m_next_vertex = this.m_vertices[1];
}

pub fn b2_chain_shape_create_chain(this: &mut B2chainShape, vertices: &[B2vec2], prev_vertex: B2vec2, next_vertex: B2vec2) {
	let count = vertices.len();
	b2_assert(this.m_vertices.len() == 0);
	b2_assert(count >= 2);
	for i in 1..count {
		// If the code crashes here, it means your vertices are too close together.
		b2_assert(
			b2_distance_vec2_squared(vertices[i - 1], vertices[i])
				> B2_LINEAR_SLOP * B2_LINEAR_SLOP,
		);
	}

	this.m_vertices = Vec::from(vertices);

	this.m_prev_vertex = prev_vertex;
	this.m_next_vertex = next_vertex;
}

pub fn b2_shape_dyn_trait_clone(this: &B2chainShape) -> Box<dyn B2shapeDynTrait> {
	return Box::new(B2chainShape::clone(&this));
}

pub fn b2_shape_dyn_trait_get_child_count(this: &B2chainShape) -> usize {
	// edge count = vertex count - 1
	return this.m_vertices.len() - 1;
}

pub fn b2_chain_shape_get_child_edge(this: &B2chainShape, edge: &mut B2edgeShape, index: usize) {
	b2_assert(index < this.m_vertices.len() - 1);
	edge.base.m_type = B2ShapeType::EEdge;
	edge.base.m_radius = this.base.m_radius;

	edge.m_vertex1 = this.m_vertices[index + 0];
	edge.m_vertex2 = this.m_vertices[index + 1];
	edge.m_one_sided = true;

	if index > 0 {
		edge.m_vertex0 = this.m_vertices[index - 1];
	} else {
		edge.m_vertex0 = this.m_prev_vertex;
	}

	if index < this.m_vertices.len() - 2 {
		edge.m_vertex3 = this.m_vertices[index + 2];
	} else {
		edge.m_vertex3 = this.m_next_vertex;
	}
}

pub fn b2_shape_dyn_trait_test_point(_this: &B2chainShape, xf: B2Transform, p: B2vec2) -> bool {
	b2_not_used(xf);
	b2_not_used(p);
	return false;
}
pub fn b2_shape_dyn_trait_ray_cast(
	this: &B2chainShape,
	output: &mut B2rayCastOutput,
	input: &B2rayCastInput,
	xf: B2Transform,
	child_index: usize,
) -> bool {
	b2_assert(child_index < this.m_vertices.len());

	let mut edge_shape = B2edgeShape::default();

	let i1: usize = child_index;
	let mut i2: usize = child_index + 1;
	if i2 == this.m_vertices.len() {
		i2 = 0;
	}

	edge_shape.m_vertex1 = this.m_vertices[i1];
	edge_shape.m_vertex2 = this.m_vertices[i2];

	return edge_shape.ray_cast(output, &input, xf, 0);
}

pub fn b2_shape_dyn_trait_compute_aabb(
	this: &B2chainShape,
	aabb: &mut B2AABB,
	xf: B2Transform,
	child_index: usize,
) {
	b2_assert(child_index < this.m_vertices.len());

	let i1: usize = child_index;
	let mut i2: usize = child_index + 1;
	if i2 == this.m_vertices.len() {
		i2 = 0;
	}

	let v1: B2vec2 = b2_mul_transform_by_vec2(xf, this.m_vertices[i1]);
	let v2: B2vec2 = b2_mul_transform_by_vec2(xf, this.m_vertices[i2]);

	let lower: B2vec2 = b2_min_vec2(v1, v2);
	let upper: B2vec2 = b2_max_vec2(v1, v2);

	let r = B2vec2::new(this.base.m_radius, this.base.m_radius);
	aabb.lower_bound = lower - r;
	aabb.upper_bound = upper + r;

}

pub fn b2_shape_dyn_trait_compute_mass(_this: &B2chainShape, mass_data: &mut B2massData, density: f32) {
	b2_not_used(density);

	mass_data.mass = 0.0;
	mass_data.center.set_zero();
	mass_data.i = 0.0;
}
