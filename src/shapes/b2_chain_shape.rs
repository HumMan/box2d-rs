use super::b2_edge_shape::B2edgeShape;
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;
use crate::private::collision::b2_chain_shape as private;
#[cfg(feature="serde_support")]
use serde::{Deserialize, Serialize};
use std::rc::Rc;

/// A chain shape is a free form sequence of line segments.
/// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
/// This provides a counter-clockwise winding like the polygon shape.
/// Connectivity information is used to create smooth collisions.
/// @warning the chain will not collide properly if there are self-intersections.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct B2chainShape {
	pub base: B2Shape,
	/// The vertices. Owned by this class.
	pub m_vertices: Vec<B2vec2>,

	pub m_prev_vertex: B2vec2,
	pub m_next_vertex: B2vec2,
}

impl Default for B2chainShape {
	fn default() -> Self {
		return inline::b2_chain_shape();
	}
}

impl B2chainShape {
	/// clear all data.
	pub fn clear(&mut self) {
		private::b2_chain_shape_clear(self);
	}

	/// create a loop. This automatically adjusts connectivity.
	/// * `vertices` - an array of vertices, these are copied
	/// * `count` - the vertex count
	pub fn create_loop(&mut self, vertices: &[B2vec2]) {
		private::b2_chain_shape_create_loop(self, vertices);
	}

	/// Create a chain with ghost vertices to connect multiple chains together.
	/// * `vertices` - an array of vertices, these are copied
	/// * `count` - the vertex count
	/// * `prev_vertex` - previous vertex from chain that connects to the start
	/// * `next_vertex` - next vertex from chain that connects to the end
	pub fn create_chain(&mut self, vertices: &[B2vec2], prev_vertex: B2vec2, next_vertex: B2vec2) {
		private::b2_chain_shape_create_chain(self, vertices, prev_vertex, next_vertex);
	}

	/// Get a child edge.
	pub fn get_child_edge(&self, edge: &mut B2edgeShape, index: usize) {
		private::b2_chain_shape_get_child_edge(self, edge, index);
	}
}

impl B2shapeDynTrait for B2chainShape {
	fn get_base(&self) -> &B2Shape {
		return &self.base;
	}

	fn get_type(&self) -> B2ShapeType {
		return self.base.get_type();
	}
	/// Implement b2Shape. Vertices are cloned using b2Alloc.
	fn clone_box(&self) -> Box<dyn B2shapeDynTrait> {
		return private::b2_shape_dyn_trait_clone(self);
	}
	fn clone_rc(&self) -> ShapePtr {
		return Rc::new(self.clone());
	}
	/// [see](B2shapeDynTrait::get_child_count)
	fn get_child_count(&self) -> usize {
		return private::b2_shape_dyn_trait_get_child_count(self);
	}

	/// This always return false.
	/// [see](B2shapeDynTrait::test_point)
	fn test_point(&self, transform: B2Transform, p: B2vec2) -> bool {
		return private::b2_shape_dyn_trait_test_point(self, transform, p);
	}

	/// Implement b2Shape.
	fn ray_cast(
		&self,
		output: &mut B2rayCastOutput,
		input: &B2rayCastInput,
		xf: B2Transform,
		child_index: usize,
	) -> bool {
		return private::b2_shape_dyn_trait_ray_cast(self, output, input, xf, child_index);
	}

	/// [see](B2shapeDynTrait::compute_aabb)
	fn compute_aabb(&self, aabb: &mut B2AABB, xf: B2Transform, child_index: usize) {
		private::b2_shape_dyn_trait_compute_aabb(self, aabb, xf, child_index);
	}

	/// Chains have zero mass.
	/// [see](B2shapeDynTrait::compute_mass)
	fn compute_mass(&self, mass_data: &mut B2massData, density: f32) {
		private::b2_shape_dyn_trait_compute_mass(self, mass_data, density);
	}
}

mod inline {
	use super::*;
	pub fn b2_chain_shape() -> B2chainShape {
		return B2chainShape {
			base: B2Shape {
				m_type: B2ShapeType::EChain,
				m_radius: B2_POLYGON_RADIUS,
			},
			m_next_vertex: B2vec2 { x: 0.0, y: 0.0 },
			m_prev_vertex: B2vec2 { x: 0.0, y: 0.0 },
			m_vertices: Vec::<B2vec2>::new(),
		};
	}
}
