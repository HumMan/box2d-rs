use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;
use crate::private::collision::b2_edge_shape as private;
use std::rc::Rc;

#[cfg(feature="serde_support")]
use serde::{Serialize, Deserialize};

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. Edges created independently are two-sided and do
/// no provide smooth movement across junctions. 
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct B2edgeShape {
	pub base: B2Shape,

	/// These are the edge vertices
	pub m_vertex1: B2vec2,
	pub m_vertex2: B2vec2,

	/// Optional adjacent vertices. These are used for smooth collision.
	pub m_vertex0: B2vec2,
	pub m_vertex3: B2vec2,
	/// Uses m_vertex0 and m_vertex3 to create smooth collision.
	pub m_one_sided: bool
}

impl Default for B2edgeShape {
	fn default() -> Self {
		return inline::b2_edge_shape();
	}
}

impl B2edgeShape {

	/// Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
	/// follows. These extra vertices are used to provide smooth movement
	/// across junctions. This also makes the collision one-sided. The edge
	/// normal points to the right looking from v1 to v2.
	pub fn set_one_sided(&mut self, v0: B2vec2, v1: B2vec2, v2: B2vec2, v3: B2vec2)
	{
		private::b2_set_one_sided(self, v0, v1, v2, v3);
	}

	/// Set this as an isolated edge. Collision is two-sided.
	pub fn set_two_sided(&mut self, v1: B2vec2, v2: B2vec2)
	{
		private::b2_set_two_sided(self, v1, v2);
	}
}

impl B2shapeDynTrait for B2edgeShape {
	fn get_base(&self) -> &B2Shape
	{
		return &self.base;
	}
	fn get_type(&self) -> B2ShapeType {
		return self.base.get_type();
	}
	/// Implement b2Shape.
	fn clone_box(&self) -> Box<dyn B2shapeDynTrait> {
		return private::b2_shape_dyn_trait_clone(self);
	}
	fn clone_rc(&self) -> ShapePtr
	{
		return Rc::new(self.clone());
	}
	/// [see](B2shapeDynTrait::get_child_count)
	fn get_child_count(&self) -> usize {
		return private::b2_shape_dyn_trait_get_child_count(self);
	}

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
		return private::b2_shape_dyn_trait_ray_cast(
			&self, output, input, xf, child_index,
		);
	}

	/// [see](B2shapeDynTrait::compute_aabb)
	fn compute_aabb(&self, aabb: &mut B2AABB, xf: B2Transform, child_index: usize) {
		private::b2_shape_dyn_trait_compute_aabb(self, aabb, xf, child_index);
	}

	/// [see](B2shapeDynTrait::compute_mass)
	fn compute_mass(&self, mass_data: &mut B2massData, density: f32) {
		private::b2_shape_dyn_trait_compute_mass(self, mass_data, density);
	}
}

mod inline
{
	use super::*;
	pub fn b2_edge_shape() -> B2edgeShape {
		return B2edgeShape {
			base: B2Shape {
				m_type: B2ShapeType::EEdge,
				m_radius: B2_POLYGON_RADIUS,
			},
			m_vertex1: B2vec2 { x: 0.0, y: 0.0 },
			m_vertex2: B2vec2 { x: 0.0, y: 0.0 },
			m_vertex0: B2vec2 { x: 0.0, y: 0.0 },
			m_vertex3: B2vec2 { x: 0.0, y: 0.0 },
			m_one_sided: false,
		};
	}
}