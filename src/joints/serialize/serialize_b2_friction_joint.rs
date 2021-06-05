use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::Rc;
use std::cell::RefCell;

use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_settings::*;

use crate::b2_world::*;
use crate::joints::b2_friction_joint::*;
use crate::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2frictionJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2frictionJointDef<D>;
}

impl<D: UserDataType> B2frictionJoinToDef<D> for B2frictionJoint<D> {
    fn get_def(&self) -> B2frictionJointDef<D> {
        return B2frictionJointDef {
            base: self.base.get_def(),
            local_anchor_a:  self.m_local_anchor_a,
            local_anchor_b:  self.m_local_anchor_b,
            max_force: self.m_max_force,
            max_torque: self.m_max_torque,
        };
    }
}

impl<D: UserDataType> Serialize for B2frictionJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2frictionJointDef", 5)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("local_anchor_a", &self.local_anchor_a)?;
        state.serialize_field("local_anchor_b", &self.local_anchor_b)?;
        state.serialize_field("max_force", &self.max_force)?;
        state.serialize_field("max_torque", &self.max_torque)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2frictionJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2frictionJointDefContext<U> {
    type Value = B2frictionJointDef<U>;

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
            max_force,
            max_torque,
        }

        struct B2frictionJointDefVisitor<D: UserDataType>(B2frictionJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2frictionJointDefVisitor<U> {
            type Value = B2frictionJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2frictionJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {

                
                let joint_def = B2frictionJointDef {
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

                    max_force: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    max_torque: seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2frictionJointDef::default();

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
                        Field::max_force => {
                            joint_def.max_force = map.next_value()?;
                        }
                        Field::max_torque => {
                            joint_def.max_torque = map.next_value()?;
                        }
                    }
                }

                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct("B2frictionJointDef", Field::VARIANTS, B2frictionJointDefVisitor(self))
    }
}