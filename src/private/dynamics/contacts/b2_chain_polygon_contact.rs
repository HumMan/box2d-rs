use crate::b2_contact::*;
use crate::b2_settings::*;
use crate::b2rs_common::UserDataType;
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_fixture::*;
use crate::b2_shape::*;
use crate::shapes::b2_edge_shape::*;
use std::rc::Rc;
use std::cell::RefCell;

pub(crate) struct B2chainAndPolygonContact<D: UserDataType>
{
	base: B2contact<D>,
}

impl<D: UserDataType> B2chainAndPolygonContact<D>
{
	pub fn create(fixture_a: FixturePtr<D>, index_a: i32, fixture_b: FixturePtr<D>, index_b: i32) -> ContactPtr<D>
	 {
		return Rc::new(RefCell::new(B2chainAndPolygonContact::new(fixture_a,index_a, fixture_b, index_b)));
	}

	pub fn new(fixture_a: FixturePtr<D>, index_a: i32, fixture_b: FixturePtr<D>, index_b: i32) -> Self
	{
		b2_assert(fixture_a.as_ref().borrow().get_type() == B2ShapeType::EChain);
		b2_assert(fixture_b.as_ref().borrow().get_type() == B2ShapeType::EPolygon);

		return Self{
			base: B2contact::new(fixture_a, index_a, fixture_b, index_b)
		};
	}
}

impl<D: UserDataType> B2contactDynTrait<D> for B2chainAndPolygonContact<D>
{
	fn get_base<'a>(&'a self) -> &'a B2contact<D>
	{
		return &self.base;
	}
	fn get_base_mut<'a>(&'a mut self) -> &'a mut B2contact<D>
	{
		return &mut self.base;
	}

	fn evaluate(&self, manifold: &mut B2manifold, xf_a: &B2Transform, xf_b: &B2Transform)
	{
		let chain_shape = self.base.m_fixture_a.as_ref().borrow().get_shape();
		let chain = chain_shape.as_chain().unwrap();
		 let mut edge = B2edgeShape::default();
		 chain.get_child_edge(&mut edge, self.base.m_index_a as usize);
		 let polygon_shape = self.base.m_fixture_b.as_ref().borrow().get_shape();
		 let polygon = polygon_shape.as_polygon().unwrap();
		 b2_collide_edge_and_polygon(manifold, &edge, xf_a, polygon, xf_b);
	}	
}
