use crate::b2_body::*;
use crate::b2_broad_phase::*;
use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_shape::*;
use crate::shapes::to_derived_shape::*;
use crate::linked_list::*;
use crate::private::dynamics::b2_fixture as private;
use super::b2_fixture::*;
use std::cell::RefCell;
use std::rc::{Rc, Weak};

use std::fmt;
use serde::{Serialize, Deserialize, Serializer, ser::{SerializeStruct, SerializeSeq}};
use serde::de;
use serde::de::{DeserializeSeed, Deserializer, Visitor, SeqAccess, MapAccess};
use strum::VariantNames;
use strum_macros::EnumVariantNames;

impl<D: UserDataType> Serialize for B2fixture<D>
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
		let mut state = serializer.serialize_struct("B2fixture", 1)?;
		state.serialize_field("m_density", &self.m_density)?;
		state.serialize_field("m_friction", &self.m_friction)?;
		state.serialize_field("m_restitution", &self.m_restitution)?;
		state.serialize_field("m_proxy_count", &self.m_density)?;
		state.serialize_field("m_filter", &self.m_filter)?;
		state.serialize_field("m_is_sensor", &self.m_is_sensor)?;
		match self.m_shape.as_ref().unwrap().as_derived()
		{
			ShapeAsDerived::AsCircle(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			},
			ShapeAsDerived::AsEdge(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			},
			ShapeAsDerived::AsPolygon(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			},
			ShapeAsDerived::AsChain(ref shape) => {
				state.serialize_field("m_shape", &shape)?;
			},
		}
        state.end()
    }
}

struct B2fixtureVisitorContext<D: UserDataType>
{
	pub(crate) m_fixture_list: LinkedList<B2fixture<D>>,
	pub(crate) m_body: BodyWeakPtr<D>,
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
		enum Field { base, m_centroid, m_count, m_vertices, m_normals };	
		
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
				let mut result = B2fixture::<U>::default();
				// result.base = seq.next_element()?
                //      .ok_or_else(|| de::Error::invalid_length(0, &self))?;
				// result.m_centroid = seq.next_element()?
				// 	.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				// result.m_count = seq.next_element()?
				// 	.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				// result.m_count=0;
				// seq.next_element_seed(VecArray{count: &mut result.m_count, array: &mut result.m_vertices})?
				// 	.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				// result.m_count=0;
				// seq.next_element_seed(VecArray{count: &mut result.m_count, array: &mut result.m_normals})?
				// 	.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				Ok(())
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut result = B2fixture::<U>::default();
                // while let Some(key) = map.next_key()? {
                //     match key {
                //         Field::base => {
                //             result.base = map.next_value()?;
				// 		}
				// 		Field::m_centroid => {
                //             result.m_centroid = map.next_value()?;
				// 		}
				// 		Field::m_count => {
                //             result.m_count = map.next_value()?;
				// 		}
				// 		Field::m_vertices => {
				// 			result.m_count=0;
				// 			map.next_value_seed(VecArray{count: &mut result.m_count, array: &mut result.m_vertices})?;
				// 		}
				// 		Field::m_normals => {
				// 			result.m_count=0;
				// 			map.next_value_seed(VecArray{count: &mut result.m_count, array: &mut result.m_normals})?;
				// 		}
                //     }
                // }
                Ok(())
            }
		}
		
		deserializer.deserialize_struct("Duration", Field::VARIANTS, B2fixtureVisitor(self))
    }
}