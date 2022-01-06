use crate::b2_collision::*;
use crate::b2_settings::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2_shape::*;
use crate::private::collision::b2_polygon_shape as private;
use std::rc::Rc;

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
/// In most cases you should not need many vertices for a convex polygon.
#[derive(Clone, Copy, Debug)]
pub struct B2polygonShape {
	pub base: B2Shape,
	pub m_centroid: B2vec2,
	pub m_vertices: [B2vec2; B2_MAX_POLYGON_VERTICES],
	pub m_normals: [B2vec2; B2_MAX_POLYGON_VERTICES],
	pub m_count: usize,
}

impl Default for B2polygonShape {
	fn default() -> Self {
		return inline::b2_polygon_shape();
	}
}

impl B2shapeDynTrait for B2polygonShape {
	fn get_base(&self) -> &B2Shape {
		return &self.base;
	}
	fn get_type(&self) -> B2ShapeType {
		return self.base.get_type();
	}
	/// Implement b2Shape.
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

	/// [see](B2shapeDynTrait::test_point)
	fn test_point(&self, transform: B2Transform, p: B2vec2) -> bool {
		return private::b2_shape_dyn_trait_test_point(self, transform, p);
	}

	/// Implement b2Shape.
	/// @note because the polygon is solid, rays that start inside do not hit because the normal is
	/// not defined.
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

	/// [see](B2shapeDynTrait::compute_mass)
	fn compute_mass(&self, mass_data: &mut B2massData, density: f32) {
		private::b2_shape_dyn_trait_compute_mass(self, mass_data, density);
	}
}

impl B2polygonShape {
	pub const STRUCT_NAME: &'static str = "B2polygonShape";
	/// create a convex hull from the given array of local points.
	/// The count must be in the range [3, B2_MAX_POLYGON_VERTICES].
	/// <p style="background:rgba(255,181,77,0.16);padding:0.75em;">
	/// <strong>Warning:</strong> the points may be re-ordered, even if they form a convex polygon
	/// </p>
	/// <p style="background:rgba(255,181,77,0.16);padding:0.75em;">
	/// <strong>Warning:</strong> collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	/// </p>
	pub fn set(&mut self, points: &[B2vec2]) {
		private::b2_polygon_shape_set(self, points);
	}

	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// * `hx` - the half-width.
	/// * `hy` - the half-height.
	pub fn set_as_box(&mut self, hx: f32, hy: f32) {
		private::b2_polygon_shape_set_as_box(self, hx, hy);
	}

	/// Build vertices to represent an oriented box.
	/// * `hx` - the half-width.
	/// * `hy` - the half-height.
	/// * `center` - the center of the box in local coordinates.
	/// * `angle` - the rotation of the box in local coordinates.
	pub fn set_as_box_angle(&mut self, hx: f32, hy: f32, center: B2vec2, angle: f32) {
		private::b2_polygon_shape_set_as_box_angle(self, hx, hy, center, angle);
	}

	/// Validate convexity. This is a very time consuming operation.
	/// 
	/// @returns true if valid
	pub fn validate(self) -> bool {
		return private::b2_polygon_shape_validate(self);
	}
}

mod inline {
	use super::*;
	pub fn b2_polygon_shape() -> B2polygonShape {
		return B2polygonShape {
			base: B2Shape {
				m_type: B2ShapeType::EPolygon,
				m_radius: B2_POLYGON_RADIUS,
			},
			m_centroid: B2vec2::zero(),
			m_count: 0,
			m_vertices: <[B2vec2; B2_MAX_POLYGON_VERTICES]>::default(),
			m_normals: <[B2vec2; B2_MAX_POLYGON_VERTICES]>::default(),
		};
	}
}
