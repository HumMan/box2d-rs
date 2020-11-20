use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_shape::*;
use crate::private::collision::b2_circle_shape as private;
use std::rc::Rc;

#[cfg(feature="serde_support")]
use serde::{Serialize, Deserialize};

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct B2circleShape {
	pub base: B2Shape,
	/// Position
	pub m_p: B2vec2,
}

impl Default for B2circleShape {
	fn default() -> Self {
		return inline::b2_circle_shape();
	}
}

/// A solid circle shape
impl B2shapeDynTrait for B2circleShape {
	fn get_base(&self) -> &B2Shape
	{
		return &self.base;
	}
	
	fn get_type(&self) -> B2ShapeType {
		return self.base.get_type();
	}
	/// Implement b2Shape.
	fn clone_box(&self) -> Box<dyn B2shapeDynTrait> {
		return private::clone(self);
	}
	
	fn clone_rc(&self) -> ShapePtr
	{
		return Rc::new(self.clone());
	}

	/// [see](B2shapeDynTrait::get_child_count)
	fn get_child_count(&self) -> usize {
		return private::get_child_count(self);
	}

	/// Implement b2Shape.
	fn test_point(&self, transform: B2Transform, p: B2vec2) -> bool {
		return private::test_point(self, transform, p);
	}

	/// Implement b2Shape.
	/// @note because the circle is solid, rays that start inside do not hit because the normal is
	/// not defined.
	fn ray_cast(
		&self,
		output: &mut B2rayCastOutput,
		input: &B2rayCastInput,
		xf: B2Transform,
		child_index: usize,
	) -> bool {
		return private::ray_cast(
			self, output, input, xf, child_index,
		);
	}

	/// [see](B2shapeDynTrait::compute_aabb)
	fn compute_aabb(&self, aabb: &mut B2AABB, xf: B2Transform, child_index: usize) {
		private::compute_aabb(
			self, aabb, xf, child_index,
		);
	}

	/// [see](B2shapeDynTrait::compute_mass)
	fn compute_mass(&self, mass_data: &mut B2massData, density: f32) {
		private::compute_mass(self, mass_data, density);
	}
}

mod inline
{
	use super::*;
	pub fn b2_circle_shape() -> B2circleShape {
		return B2circleShape {
			base: B2Shape {
				m_type: B2ShapeType::ECircle,
				m_radius: 0.0,
			},
			m_p: B2vec2::zero(),
		};
	}
}