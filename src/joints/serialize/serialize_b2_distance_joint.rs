use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::Rc;
use std::cell::RefCell;

use crate::b2_body::*;
use crate::b2rs_common::UserDataType;

use crate::joints::b2_distance_joint::*;
use crate::serialize::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2DistanceJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2distanceJointDef<D>;
}

impl<D: UserDataType> B2DistanceJoinToDef<D> for B2distanceJoint<D> {
    fn get_def(&self) -> B2distanceJointDef<D> {
        return B2distanceJointDef {
            base: self.base.get_def(),
            local_anchor_a:  self.m_local_anchor_a,
            local_anchor_b:  self.m_local_anchor_b,
            length: self.m_length,
            min_length: self.m_min_length,
            max_length: self.m_max_length,
            stiffness: self.m_stiffness,
            damping: self.m_damping
        };
    }
}

impl<D: UserDataType> Serialize for B2distanceJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2distanceJointDef", 6)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("local_anchor_a", &self.local_anchor_a)?;
        state.serialize_field("local_anchor_b", &self.local_anchor_b)?;
        state.serialize_field("length", &self.length)?;
        state.serialize_field("min_length", &self.min_length)?;
        state.serialize_field("max_length", &self.max_length)?;
        state.serialize_field("stiffness", &self.stiffness)?;
        state.serialize_field("damping", &self.damping)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2distanceJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2distanceJointDefContext<U> {
    type Value = B2distanceJointDef<U>;

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        #[derive(EnumVariantNames)]
        #[allow(non_camel_case_types)]
        enum Field {
            base,
            local_anchor_a,
            local_anchor_b,
            length,
            min_length,
            max_length,
            stiffness,
            damping
        }

        struct B2distanceJointDefVisitor<D: UserDataType>(B2distanceJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2distanceJointDefVisitor<U> {
            type Value = B2distanceJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2distanceJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {

                
                let joint_def = B2distanceJointDef {
                    base: seq.next_element_seed(B2jointDefVisitorContext {
                        m_body_array: self.0.m_body_array.clone(),
                    })?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    local_anchor_a: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    local_anchor_b: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    length: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    min_length: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    max_length: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    stiffness: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    damping: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2distanceJointDef::default();

                while let Some(key) = map.next_key()? {
                    match key {
                        Field::base => {
                            joint_def.base = map.next_value_seed(B2jointDefVisitorContext {
                                m_body_array: self.0.m_body_array.clone(),
                            })?;
                        }
                        Field::local_anchor_a => {
                            joint_def.local_anchor_a = map.next_value()?;
                        }
                        Field::local_anchor_b => {
                            joint_def.local_anchor_b = map.next_value()?;
                        }
                        Field::length => {
                            joint_def.length = map.next_value()?;
                        }
                        Field::min_length => {
                            joint_def.min_length = map.next_value()?;
                        }
                        Field::max_length => {
                            joint_def.max_length = map.next_value()?;
                        }
                        Field::stiffness => {
                            joint_def.stiffness = map.next_value()?;
                        }
                        Field::damping => {
                            joint_def.damping = map.next_value()?;
                        }
                    }
                }

                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct("B2distanceJointDef", Field::VARIANTS, B2distanceJointDefVisitor(self))
    }
}