use crate::b2_collision::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_shape::*;
use super::b2_polygon_shape::*;
use crate::serialize_context;
use crate::private::collision::b2_polygon_shape as private;
use std::rc::Rc;

use serde::{Serialize, Deserialize, Serializer, ser::{SerializeStruct, SerializeSeq}};
use std::fmt;
use serde::de;
use serde::de::{DeserializeSeed, Deserializer, Visitor, SeqAccess, MapAccess};
use strum::VariantNames;
use strum_macros::{EnumString,EnumVariantNames, AsRefStr};

#[derive(Deserialize)]
#[serde(field_identifier, rename_all = "lowercase")]
#[derive(EnumString, EnumVariantNames, AsRefStr)]
enum Field {
	base,
	m_centroid,
	m_count,
	m_vertices,
	m_normals,
}

impl Serialize for B2polygonShape
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
		let mut state = serializer.serialize_struct(B2polygonShape::STRUCT_NAME, 19)?;
		state.serialize_field(Field::base.as_ref(), &self.base)?;
		state.serialize_field(Field::m_centroid.as_ref(), &self.m_centroid)?;
		state.serialize_field("m_count", &self.m_count)?;
		{
			struct VerticesContext;
			impl Serialize for serialize_context::WithContext<'_, B2polygonShape, VerticesContext>
			{
				fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
				where
					S: Serializer,
				{
					let len = self.context.m_count;
					let mut state = serializer.serialize_seq(Some(len))?;
					for i in 0..len
					{
						state.serialize_element(&self.context.m_vertices[i])?;
					}
					state.end()
				}
			}
			
			state.serialize_field("m_vertices", &serialize_context::WithContext::<'_, B2polygonShape, VerticesContext>::new(&self))?;

			struct NormalsContext;
			impl Serialize for serialize_context::WithContext<'_, B2polygonShape, NormalsContext>
			{
				fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
				where
					S: Serializer,
				{
					let len = self.context.m_count;
					let mut state = serializer.serialize_seq(Some(len))?;
					for i in 0..len
					{
						state.serialize_element(&self.context.m_normals[i])?;
					}
					state.end()
				}
			}
			
			state.serialize_field("m_normals", &serialize_context::WithContext::<'_, B2polygonShape, NormalsContext>::new(&self))?;
		}
		state.end()
	}
}

impl<'de> Deserialize<'de> for B2polygonShape {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {		
		struct VecArray<'ctx>
		{
			count: &'ctx mut usize,
			array: &'ctx mut [B2vec2; B2_MAX_POLYGON_VERTICES]
		}

		impl<'de> DeserializeSeed<'de> for VecArray<'_>
		{
			// The return type of the `deserialize` method. This implementation
			// appends onto an existing vector but does not create any new data
			// structure, so the return type is ().
			type Value = ();

			fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
			where
				D: Deserializer<'de>,
			{
				// Visitor implementation that will walk an inner array of the JSON
				// input.
				struct VecArrayVisitor<'a>(VecArray<'a>);

				impl<'de> Visitor<'de> for VecArrayVisitor<'_>
				{
					type Value = ();

					fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
						write!(formatter, "an array of integers")
					}

					fn visit_seq<A>(self, mut seq: A) -> Result<(), A::Error>
					where
						A: SeqAccess<'de>,
					{
						// Visit each element in the inner array and push it onto
						// the existing vector.
						while let Some(elem) = seq.next_element()? {
							self.0.array[*self.0.count]=elem;
							*self.0.count+=1;
						}
						Ok(())
					}
				}

				deserializer.deserialize_seq(VecArrayVisitor(self))
			}
		}

        struct B2polygonShapeVisitor;

        impl<'de> Visitor<'de> for B2polygonShapeVisitor {
            type Value = B2polygonShape;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2polygonShape")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<B2polygonShape, V::Error>
            where
                V: SeqAccess<'de>,
            {
				let mut result = B2polygonShape::default();
				result.base = seq.next_element()?
                     .ok_or_else(|| de::Error::invalid_length(0, &self))?;
				result.m_centroid = seq.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				result.m_count = seq.next_element()?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				result.m_count=0;
				seq.next_element_seed(VecArray{count: &mut result.m_count, array: &mut result.m_vertices})?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				result.m_count=0;
				seq.next_element_seed(VecArray{count: &mut result.m_count, array: &mut result.m_normals})?
					.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				Ok(result)
            }

            fn visit_map<V>(self, mut map: V) -> Result<B2polygonShape, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut result = B2polygonShape::default();
                while let Some(key) = map.next_key::<Field>()? {
                    match key {
                        Field::base => {
                            result.base = map.next_value()?;
						}
						Field::m_centroid => {
                            result.m_centroid = map.next_value()?;
						}
						Field::m_count => {
                            result.m_count = map.next_value()?;
						}
						Field::m_vertices => {
							result.m_count=0;
							map.next_value_seed(VecArray{count: &mut result.m_count, array: &mut result.m_vertices})?;
						}
						Field::m_normals => {
							result.m_count=0;
							map.next_value_seed(VecArray{count: &mut result.m_count, array: &mut result.m_normals})?;
						}
                    }
                }
                Ok(result)
            }
        }

        deserializer.deserialize_struct(B2polygonShape::STRUCT_NAME, Field::VARIANTS, B2polygonShapeVisitor)
    }
}