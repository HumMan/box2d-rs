use crate::b2_fixture::*;
use crate::b2_body::*;
use crate::b2rs_common::UserDataType;
use crate::b2_shape::*;
use crate::shapes::b2_chain_shape::*;
use crate::shapes::b2_circle_shape::*;
use crate::shapes::b2_edge_shape::*;
use crate::shapes::b2_polygon_shape::*;
use crate::shapes::b2rs_to_derived_shape::*;
use std::cell::RefCell;
use std::rc::{Rc};

use serde::de;
use serde::de::{DeserializeSeed, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{
	ser::{SerializeStruct},
	Deserialize, Serialize, Serializer,
};
use std::fmt;
use strum::VariantNames;
use strum_macros::EnumVariantNames;

impl<D: UserDataType> Serialize for B2fixture<D> {
	fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
	where
		S: Serializer,
	{
		let mut state = serializer.serialize_struct("B2fixture", 7)?;
		state.serialize_field("m_friction", &self.m_friction)?;
		state.serialize_field("m_restitution", &self.m_restitution)?;
		state.serialize_field("m_restitution_threshold", &self.m_restitution_threshold)?;
		state.serialize_field("m_density", &self.m_density)?;
		state.serialize_field("m_is_sensor", &self.m_is_sensor)?;
		state.serialize_field("m_filter", &self.m_filter)?;
		state.serialize_field("m_shape_type", &self.m_shape.as_ref().unwrap().get_type())?;
		match self.m_shape.as_ref().unwrap().as_derived() {
			ShapeAsDerived::AsCircle(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			}
			ShapeAsDerived::AsEdge(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			}
			ShapeAsDerived::AsPolygon(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			}
			ShapeAsDerived::AsChain(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			}
		}
		state.end()
	}
}

struct B2fixtureVisitorContext<D: UserDataType> {
	pub(crate) body: BodyWeakPtr<D>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2fixtureVisitorContext<U> {
	type Value = ();

	fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
	where
		D: Deserializer<'de>,
	{
		#[derive(Deserialize)]
		#[serde(field_identifier, rename_all = "lowercase")]
		#[derive(EnumVariantNames)]
        #[allow(non_camel_case_types)]
		enum Field {
			m_friction,
			m_restitution,
			m_restitution_threshold,
			m_density,
			m_is_sensor,
			m_filter,
			m_shape_type,
			m_shape,
		}
		struct B2fixtureVisitor<D: UserDataType>(B2fixtureVisitorContext<D>);

		impl<'de, U: UserDataType> Visitor<'de> for B2fixtureVisitor<U> {
			type Value = ();

			fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
				formatter.write_str("struct B2fixture")
			}

			fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
			where
				V: SeqAccess<'de>,
			{
				let body = self.0.body.upgrade().unwrap();
				let mut definition = B2fixtureDef::<U>::default();

				definition.friction = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				definition.restitution = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				definition.restitution_threshold = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				definition.density = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				definition.is_sensor = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				definition.filter = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				let shape_type = seq
					.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;

				match shape_type {
					B2ShapeType::EChain => {
						let shape: B2chainShape = seq
							.next_element()?
							.ok_or_else(|| de::Error::invalid_length(0, &self))?;
						definition.shape = Some(Rc::new(RefCell::new(shape)));
					}
					B2ShapeType::ECircle => {
						let shape: B2circleShape = seq
							.next_element()?
							.ok_or_else(|| de::Error::invalid_length(0, &self))?;
						definition.shape = Some(Rc::new(RefCell::new(shape)));
					}
					B2ShapeType::EEdge => {
						let shape: B2edgeShape = seq
							.next_element()?
							.ok_or_else(|| de::Error::invalid_length(0, &self))?;
						definition.shape = Some(Rc::new(RefCell::new(shape)));
					}
					B2ShapeType::EPolygon => {
						let shape: B2polygonShape = seq
							.next_element()?
							.ok_or_else(|| de::Error::invalid_length(0, &self))?;
						definition.shape = Some(Rc::new(RefCell::new(shape)));
					}
					_ => panic!(),
				}
				B2body::create_fixture(body, &definition);
				Ok(())
			}

			fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
			where
				V: MapAccess<'de>,
			{
				let body = self.0.body.upgrade().unwrap();
				let mut definition = B2fixtureDef::<U>::default();
				let mut shape_type = B2ShapeType::EChain;
				while let Some(key) = map.next_key()? {
					match key {
						Field::m_friction => {
							definition.friction = map.next_value()?;
						}
						Field::m_restitution => {
							definition.restitution = map.next_value()?;
						}
						Field::m_restitution_threshold => {
							definition.restitution_threshold = map.next_value()?;
						}
						Field::m_density => {
							definition.density = map.next_value()?;
						}
						Field::m_is_sensor => {
							definition.is_sensor = map.next_value()?;
						}
						Field::m_filter => {
							definition.filter = map.next_value()?;
						}
						Field::m_shape_type => {
							shape_type = map.next_value()?;
						}
						Field::m_shape => match shape_type {
							B2ShapeType::EChain => {
								let shape: B2chainShape = map.next_value()?;
								definition.shape = Some(Rc::new(RefCell::new(shape)));
							}
							B2ShapeType::ECircle => {
								let shape: B2circleShape = map.next_value()?;
								definition.shape = Some(Rc::new(RefCell::new(shape)));
							}
							B2ShapeType::EEdge => {
								let shape: B2edgeShape = map.next_value()?;
								definition.shape = Some(Rc::new(RefCell::new(shape)));
							}
							B2ShapeType::EPolygon => {
								let shape: B2polygonShape = map.next_value()?;
								definition.shape = Some(Rc::new(RefCell::new(shape)));
							}
							_ => panic!(),
						},
					}
				}
				B2body::create_fixture(body, &definition);
				Ok(())
			}
		}
		deserializer.deserialize_struct("Duration", Field::VARIANTS, B2fixtureVisitor(self))
	}
}

pub(crate) struct B2fixtureListVisitorContext<D: UserDataType> {
	pub(crate) body: BodyWeakPtr<D>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2fixtureListVisitorContext<U> {
	type Value = ();

	fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
	where
		D: Deserializer<'de>,
	{
		struct B2bodyVisitor<D: UserDataType>(B2fixtureListVisitorContext<D>);

		impl<'de, U: UserDataType> Visitor<'de> for B2bodyVisitor<U> {
			type Value = ();

			fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
				formatter.write_str("struct B2fixture")
			}

			fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
			where
				V: SeqAccess<'de>,
			{
				while let Some(_elem) = seq.next_element_seed(B2fixtureVisitorContext {
					body: self.0.body.clone(),
				})? {}
				Ok(())
			}
		}
		deserializer.deserialize_seq(B2bodyVisitor(self))
	}
}
