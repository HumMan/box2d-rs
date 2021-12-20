use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::cell::RefCell;
use std::rc::Rc;

use crate::b2_body::*;
use crate::b2rs_common::UserDataType;

use crate::joints::b2_rope_joint::*;
use crate::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2ropeJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2ropeJointDef<D>;
}

impl<D: UserDataType> B2ropeJoinToDef<D> for B2ropeJoint<D> {
    fn get_def(&self) -> B2ropeJointDef<D> {
        return B2ropeJointDef {
            base: self.base.get_def(),
            local_anchor_a: self.m_local_anchor_a,
            local_anchor_b: self.m_local_anchor_b,
            max_length: self.m_max_length,
        };
    }
}

impl<D: UserDataType> Serialize for B2ropeJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2ropeJointDef", 4)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("local_anchor_a", &self.local_anchor_a)?;
        state.serialize_field("local_anchor_b", &self.local_anchor_b)?;
        state.serialize_field("max_length", &self.max_length)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2ropeJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2ropeJointDefContext<U> {
    type Value = B2ropeJointDef<U>;

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
            max_length,
        }

        struct B2ropeJointDefVisitor<D: UserDataType>(B2ropeJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2ropeJointDefVisitor<U> {
            type Value = B2ropeJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2ropeJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let joint_def = B2ropeJointDef {
                    base: seq
                        .next_element_seed(B2jointDefVisitorContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    local_anchor_a: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    local_anchor_b: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                        max_length: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2ropeJointDef::default();
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
                        Field::max_length => {
                            joint_def.max_length = map.next_value()?;
                        }
                    }
                }

                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct(
            "B2ropeJointDef",
            Field::VARIANTS,
            B2ropeJointDefVisitor(self),
        )
    }
}
