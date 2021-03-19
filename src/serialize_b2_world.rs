use serde::de::{self, Deserializer, Error, MapAccess, SeqAccess, Visitor};
use serde::{
    ser::{SerializeStruct},
    Deserialize, Serialize, Serializer,
};
use std::fmt;
use std::marker::PhantomData;
use std::rc::{Rc};


use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_world::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

use crate::serialize_b2_body::*;

impl<D: UserDataType> Serialize for B2world<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2world", 2)?;

        state.serialize_field("m_gravity", &self.m_gravity)?;
        state.serialize_field("m_bodies_list", &self.m_body_list)?;
        state.end()
    }
}

pub struct B2worldDeserializeResult<U: UserDataType> {
    pub world: B2worldPtr<U>,
}

impl<'de, U: UserDataType> Deserialize<'de> for B2worldDeserializeResult<U> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        //TODO_humman хорошо бы этот список сразу получить и использовать в Serialize тоже
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        #[derive(EnumVariantNames)]
        enum Field {
            m_gravity,
            m_bodies_list,
        };
        #[derive(Default)]
        struct B2worldVisitor<U: UserDataType> {
            marker: PhantomData<U>,
        }

        impl<'de, U: UserDataType> Visitor<'de> for B2worldVisitor<U> {
            type Value = B2worldPtr<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2bodyDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let m_gravity: B2vec2 = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let result = B2world::new(m_gravity);
               
                seq.next_element_seed(B2bodyVisitorContext {
                    m_world: Rc::downgrade(&result),
                })?
                .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                Ok(result)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut m_gravity = B2vec2::zero();
                let mut result = Err(V::Error::custom("m_bodies_list"));
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::m_gravity => {
                            m_gravity = map.next_value()?;
                        }
                        Field::m_bodies_list => {
                            let world = B2world::<U>::new(m_gravity);
                            map.next_value_seed(B2bodyVisitorContext {
                                m_world: Rc::downgrade(&world),
                            })?;
                            result = Ok(world);
                        }
                    }
                }
                result
            }
        }

        let result =
            deserializer.deserialize_struct("B2world", Field::VARIANTS, B2worldVisitor::default());
        Ok(B2worldDeserializeResult { world: result? })
    }
}
