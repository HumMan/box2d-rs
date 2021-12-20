use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::cell::RefCell;
use std::rc::Rc;

use crate::b2_body::*;
use crate::b2rs_common::UserDataType;

use crate::joints::b2_pulley_joint::*;
use crate::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2pulleyJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2pulleyJointDef<D>;
}

impl<D: UserDataType> B2pulleyJoinToDef<D> for B2pulleyJoint<D> {
    fn get_def(&self) -> B2pulleyJointDef<D> {
        return B2pulleyJointDef {
            base: self.base.get_def(),
            ground_anchor_a: self.m_ground_anchor_a,
            ground_anchor_b: self.m_ground_anchor_b,
            local_anchor_a: self.m_local_anchor_a,
            local_anchor_b: self.m_local_anchor_b,
            length_a: self.m_length_a,
            length_b: self.m_length_b,
            ratio: self.m_ratio,
        };
    }
}

impl<D: UserDataType> Serialize for B2pulleyJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2pulleyJointDef", 8)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("ground_anchor_a", &self.ground_anchor_a)?;
        state.serialize_field("ground_anchor_b", &self.ground_anchor_b)?;
        state.serialize_field("local_anchor_a", &self.local_anchor_a)?;
        state.serialize_field("local_anchor_b", &self.local_anchor_b)?;
        state.serialize_field("length_a", &self.length_a)?;
        state.serialize_field("length_b", &self.length_b)?;
        state.serialize_field("ratio", &self.ratio)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2pulleyJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2pulleyJointDefContext<U> {
    type Value = B2pulleyJointDef<U>;

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
            ground_anchor_a,
            ground_anchor_b,
            local_anchor_a,
            local_anchor_b,
            length_a,
            length_b,
            ratio,
        }

        struct B2pulleyJointDefVisitor<D: UserDataType>(B2pulleyJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2pulleyJointDefVisitor<U> {
            type Value = B2pulleyJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2pulleyJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let joint_def = B2pulleyJointDef {
                    base: seq
                        .next_element_seed(B2jointDefVisitorContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    ground_anchor_a: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    ground_anchor_b: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    local_anchor_a: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    local_anchor_b: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    length_a: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    length_b: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    ratio: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2pulleyJointDef::default();
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::base => {
                            joint_def.base = map.next_value_seed(B2jointDefVisitorContext {
                                m_body_array: self.0.m_body_array.clone(),
                            })?;
                        }
                        Field::ground_anchor_a => {
                            joint_def.ground_anchor_a = map.next_value()?;
                        }
                        Field::ground_anchor_b => {
                            joint_def.ground_anchor_b = map.next_value()?;
                        }
                        Field::local_anchor_a => {
                            joint_def.local_anchor_a = map.next_value()?;
                        }
                        Field::local_anchor_b => {
                            joint_def.local_anchor_b = map.next_value()?;
                        }
                        Field::length_a => {
                            joint_def.length_a = map.next_value()?;
                        }
                        Field::length_b => {
                            joint_def.length_b = map.next_value()?;
                        }
                        Field::ratio => {
                            joint_def.ratio = map.next_value()?;
                        }
                    }
                }

                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct(
            "B2pulleyJointDef",
            Field::VARIANTS,
            B2pulleyJointDefVisitor(self),
        )
    }
}
