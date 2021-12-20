use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::Rc;
use std::cell::RefCell;

use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2rs_common::UserDataType;

use crate::b2_world::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2JoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2jointDef<D>;
}

impl<D: UserDataType> B2JoinToDef<D> for B2joint<D> {
    fn get_def(&self) -> B2jointDef<D> {
        return B2jointDef {
            jtype: self.m_type,
            user_data: self.m_user_data.clone(),
            body_a: Some(self.m_body_a.clone()),
            body_b: Some(self.m_body_b.clone()),
            collide_connected: self.m_collide_connected,
        };
    }
}

impl<D: UserDataType> Serialize for B2jointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2jointDef", 5)?;

        state.serialize_field("jtype", &self.jtype)?;
        state.serialize_field("user_data", &self.user_data)?;
        state.serialize_field(
            "body_a",
            &self.body_a.as_ref().unwrap().borrow().m_island_index,
        )?;
        state.serialize_field(
            "body_b",
            &self.body_b.as_ref().unwrap().borrow().m_island_index,
        )?;
        state.serialize_field("collide_connected", &self.collide_connected)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2jointDefVisitorContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2jointDefVisitorContext<U> {
    type Value = B2jointDef<U>;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        #[derive(EnumVariantNames)]
        #[allow(non_camel_case_types)]
        enum Field {
            jtype,
            user_data,
            body_a,
            body_b,
            collide_connected,
        }

        struct B2JointDefVisitor<D: UserDataType>(B2jointDefVisitorContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2JointDefVisitor<U> {
            type Value = B2jointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2fixture")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let bodies = &self.0.m_body_array;
                
                let joint_def = B2jointDef {
                    jtype: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                    user_data: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                    body_a: {
                        let body_index: i32 =  seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                        Some(bodies.borrow()[body_index as usize].clone())
                    },
                    body_b: {
                        let body_index: i32 =  seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                        Some(bodies.borrow()[body_index as usize].clone())
                    },
                    collide_connected: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2jointDef {
                    jtype: B2jointType::EUnknownJoint,
                    user_data: None,
                    body_a: None,
                    body_b: None,
                    collide_connected: false,
                };
                let bodies = &self.0.m_body_array;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::jtype => {
                            joint_def.jtype = map.next_value()?;
                        }
                        Field::user_data => {
                            joint_def.user_data = map.next_value()?;
                        }
                        Field::body_a => {
                            let body_index: i32 = map.next_value()?;
                            joint_def.body_a = Some(bodies.borrow()[body_index as usize].clone());
                        }
                        Field::body_b => {
                            let body_index: i32 = map.next_value()?;
                            joint_def.body_b = Some(bodies.borrow()[body_index as usize].clone());
                        }
                        Field::collide_connected => {
                            joint_def.collide_connected = map.next_value()?;
                        }
                    }
                }
                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct("B2jointDef", Field::VARIANTS, B2JointDefVisitor(self))
    }
}
