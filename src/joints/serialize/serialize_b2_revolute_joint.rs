use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::Rc;

use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2_settings::*;

use crate::joints::b2_revolute_joint::*;
use crate::b2_world::*;
use crate::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2RevoluteJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2revoluteJointDef<D>;
}

impl<D: UserDataType> B2RevoluteJoinToDef<D> for B2revoluteJoint<D> {
    fn get_def(&self) -> B2revoluteJointDef<D> {
        return B2revoluteJointDef {
            base: self.base.get_def(),
            local_anchor_a: self.m_local_anchor_a,
            local_anchor_b: self.m_local_anchor_b,
            reference_angle: self.m_reference_angle,
            enable_limit: self.m_enable_limit,
            lower_angle: self.m_lower_angle,
            upper_angle: self.m_upper_angle,
            enable_motor: self.m_enable_motor,
            motor_speed: self.m_motor_speed,
            max_motor_torque: self.m_max_motor_torque,
        };
    }
}

impl<D: UserDataType> Serialize for B2revoluteJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2revoluteJointDef", 10)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("local_anchor_a", &self.local_anchor_a)?;
        state.serialize_field("local_anchor_b", &self.local_anchor_b)?;
        state.serialize_field("reference_angle", &self.reference_angle)?;
        state.serialize_field("enable_limit", &self.enable_limit)?;
        state.serialize_field("lower_angle", &self.lower_angle)?;
        state.serialize_field("upper_angle", &self.upper_angle)?;
        state.serialize_field("enable_motor", &self.enable_motor)?;
        state.serialize_field("motor_speed", &self.motor_speed)?;
        state.serialize_field("max_motor_torque", &self.max_motor_torque)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2revoluteJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Vec<BodyPtr<D>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2revoluteJointDefContext<U> {
    type Value = B2revoluteJointDef<U>;

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
            reference_angle,
            enable_limit,
            lower_angle,
            upper_angle,
            enable_motor,
            motor_speed,
            max_motor_torque,
        }

        struct B2revoluteJointDefVisitor<D: UserDataType>(B2revoluteJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2revoluteJointDefVisitor<U> {
            type Value = B2revoluteJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2revoluteJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let joint_def = B2revoluteJointDef {
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

                    reference_angle: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    enable_limit: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    lower_angle: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    upper_angle: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    enable_motor: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    motor_speed: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    max_motor_torque: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,
                };

                Ok(joint_def)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut joint_def = B2revoluteJointDef::default();
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
                        Field::reference_angle => {
                            joint_def.reference_angle = map.next_value()?;
                        }
                        Field::enable_limit => {
                            joint_def.enable_limit = map.next_value()?;
                        }
                        Field::lower_angle => {
                            joint_def.lower_angle = map.next_value()?;
                        }
                        Field::upper_angle => {
                            joint_def.upper_angle = map.next_value()?;
                        }
                        Field::enable_motor => {
                            joint_def.enable_motor = map.next_value()?;
                        }
                        Field::motor_speed => {
                            joint_def.motor_speed = map.next_value()?;
                        }
                        Field::max_motor_torque => {
                            joint_def.max_motor_torque = map.next_value()?;
                        }
                    }
                }

                Ok(joint_def)
            }
        }

        deserializer.deserialize_struct(
            "B2revoluteJointDef",
            Field::VARIANTS,
            B2revoluteJointDefVisitor(self),
        )
    }
}
