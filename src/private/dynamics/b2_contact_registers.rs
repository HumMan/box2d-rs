
use crate::b2_contact::*;
use crate::b2_fixture::*;
use crate::b2rs_common::UserDataType;
use crate::b2_shape::*;

use super::contacts::b2_chain_circle_contact::*;
use super::contacts::b2_chain_polygon_contact::*;
use super::contacts::b2_circle_contact::*;
use super::contacts::b2_edge_circle_contact::*;
use super::contacts::b2_edge_polygon_contact::*;
use super::contacts::b2_polygon_circle_contact::*;
use super::contacts::b2_polygon_contact::*;

pub(crate) type B2contactCreateFcn<T> = fn(
	fixture_a: FixturePtr<T>,
	index_a: i32,
	fixture_b: FixturePtr<T>,
	index_b: i32,
) -> ContactPtr<T>;
//pub(crate) type B2contactDestroyFcn<T> = fn(contact: ContactPtr<T>);

#[derive(Default)]
pub(crate) struct B2contactRegister<T: UserDataType> {
	pub(crate) create_fcn: Option<B2contactCreateFcn<T>>,
	//pub(crate) destroyFcn: Option<B2contactDestroyFcn<T>>,
	pub(crate) primary: bool,
}

impl<D: UserDataType> Default for ContactRegisters<D> {
	fn default() -> Self {

		let def_value = || <[B2contactRegister<D>; B2ShapeType::ETypeCount as usize]>::default();
		let mut result = Self
		{
			s_registers: [def_value(),def_value(),def_value(),def_value()]
		};
		ContactRegisters::initialize_registers(&mut result);
		return result;
	}
}

pub(crate) struct ContactRegisters<D: UserDataType>
{
	pub(crate) s_registers:  [[B2contactRegister<D>; B2ShapeType::ETypeCount as usize];
	B2ShapeType::ETypeCount as usize]
}

impl<D: UserDataType> ContactRegisters<D>
{
	fn add_type(
		self_: &mut ContactRegisters<D>,
		create_fcn: B2contactCreateFcn<D>, // destoryFcn: B2contactDestroyFcn<D>,
		type1: B2ShapeType,
		type2: B2ShapeType,
	) {
		self_.s_registers[type1 as usize][type2 as usize].create_fcn = Some(create_fcn);
		//self_.s_registers.0[type1 as usize][type2 as usize].destroyFcn = Some(destoryFcn);
		self_.s_registers[type1 as usize][type2 as usize].primary = true;

		if type1 != type2 {
			self_.s_registers[type2 as usize][type1 as usize].create_fcn = Some(create_fcn);
			//self_.s_registers.0[type2 as usize][type1 as usize].destroyFcn = Some(destoryFcn);
			self_.s_registers[type2 as usize][type1 as usize].primary = false;
		}
	}
	pub(crate) fn initialize_registers(self_: &mut ContactRegisters<D>) {
		Self::add_type(self_,
			B2circleContact::create,
			B2ShapeType::ECircle,
			B2ShapeType::ECircle,
		);
		Self::add_type(self_,
			B2polygonAndCircleContact::create,
			B2ShapeType::EPolygon,
			B2ShapeType::ECircle,
		);
		Self::add_type(self_,
			B2polygonContact::create,
			B2ShapeType::EPolygon,
			B2ShapeType::EPolygon,
		);
		Self::add_type(self_,
			B2edgeAndCircleContact::create,
			B2ShapeType::EEdge,
			B2ShapeType::ECircle,
		);
		Self::add_type(self_,
			B2edgeAndPolygonContact::create,
			B2ShapeType::EEdge,
			B2ShapeType::EPolygon,
		);
		Self::add_type(self_,
			B2chainAndCircleContact::create,
			B2ShapeType::EChain,
			B2ShapeType::ECircle,
		);
		Self::add_type(self_,
			B2chainAndPolygonContact::create,
			B2ShapeType::EChain,
			B2ShapeType::EPolygon,
		);
	}
}