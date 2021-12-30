use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::cell::RefCell;
use std::rc::Rc;

use crate::b2_body::*;
use crate::b2rs_common::UserDataType;

use crate::joints::b2_motor_joint::*;
use crate::serialize::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2motorJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2motorJointDef<D>;
}

impl<D: UserDataType> B2motorJoinToDef<D> for B2motorJoint<D> {
    fn get_def(&self) -> B2motorJointDef<D> {
        return B2motorJointDef {
            base: self.base.get_def(),
            linear_offset: self.m_linear_offset,
            angular_offset: self.m_angular_offset,
            max_force: self.m_max_force,
            max_torque: self.m_max_torque,
            correction_factor: self.m_correction_factor,
        };
    }
}

impl<D: UserDataType> Serialize for B2motorJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2motorJointDef", 6)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("linear_offset", &self.linear_offset)?;
        state.serialize_field("angular_offset", &self.angular_offset)?;
        state.serialize_field("max_force", &self.max_force)?;
        state.serialize_field("max_torque", &self.max_torque)?;
        state.serialize_field("correction_factor", &self.correction_factor)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2motorJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2motorJointDefContext<U> {
    type Value = B2motorJointDef<U>;

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
            linear_offset,
            angular_offset,
            max_force,
            max_torque,
            correction_factor,
        }

        struct B2motorJointDefVisitor<D: UserDataType>(B2motorJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2motorJointDefVisitor<U> {
            type Value = B2motorJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2motorJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let joint_def = B2motorJointDef {
                    base: seq
                        .next_element_seed(B2jointDefVisitorContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    linear_offset: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    angular_offset: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    max_force: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    max_torque: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    correction_factor: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2motorJointDef::default();
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::base => {
                            joint_def.base = map.next_value_seed(B2jointDefVisitorContext {
                                m_body_array: self.0.m_body_array.clone(),
                            })?;
                        }
                        Field::linear_offset => {
                            joint_def.linear_offset = map.next_value()?;
                        }
                        Field::angular_offset => {
                            joint_def.angular_offset = map.next_value()?;
                        }
                        Field::max_force => {
                            joint_def.max_force = map.next_value()?;
                        }
                        Field::max_torque => {
                            joint_def.max_torque = map.next_value()?;
                        }
                        Field::correction_factor => {
                            joint_def.correction_factor = map.next_value()?;
                        }
                    }
                }

                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct(
            "B2motorJointDef",
            Field::VARIANTS,
            B2motorJointDefVisitor(self),
        )
    }
}
