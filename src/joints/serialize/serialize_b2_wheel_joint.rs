use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::cell::RefCell;
use std::rc::Rc;

use crate::b2_body::*;
use crate::b2_settings::*;

use crate::joints::b2_wheel_joint::*;
use crate::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2wheelJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2wheelJointDef<D>;
}

impl<D: UserDataType> B2wheelJoinToDef<D> for B2wheelJoint<D> {
    fn get_def(&self) -> B2wheelJointDef<D> {
        return B2wheelJointDef {
            base: self.base.get_def(),
            local_anchor_a: self.m_local_anchor_a,
            local_anchor_b: self.m_local_anchor_b,
            local_axis_a: self.m_local_xaxis_a,
            enable_limit: self.m_enable_limit,
            lower_translation: self.m_lower_translation,
            upper_translation: self.m_upper_translation,
            enable_motor: self.m_enable_motor,
            motor_speed: self.m_motor_speed,
            max_motor_torque: self.m_max_motor_torque,
            stiffness: self.m_stiffness,
            damping: self.m_damping,
        };
    }
}

impl<D: UserDataType> Serialize for B2wheelJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2wheelJointDef", 12)?;
        state.serialize_field("base", &self.base)?;
        state.serialize_field("local_anchor_a", &self.local_anchor_a)?;
        state.serialize_field("local_anchor_b", &self.local_anchor_b)?;
        state.serialize_field("local_axis_a", &self.local_axis_a)?;
        state.serialize_field("enable_limit", &self.enable_limit)?;
        state.serialize_field("lower_translation", &self.lower_translation)?;
        state.serialize_field("upper_translation", &self.upper_translation)?;
        state.serialize_field("enable_motor", &self.enable_motor)?;
        state.serialize_field("motor_speed", &self.motor_speed)?;
        state.serialize_field("max_motor_torque", &self.max_motor_torque)?;
        state.serialize_field("stiffness", &self.stiffness)?;
        state.serialize_field("damping", &self.damping)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2wheelJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2wheelJointDefContext<U> {
    type Value = B2wheelJointDef<U>;

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
            local_axis_a,
            enable_limit,
            lower_translation,
            upper_translation,
            enable_motor,
            motor_speed,
            max_motor_torque,
            stiffness,
            damping,
        }

        struct B2wheelJointDefVisitor<D: UserDataType>(B2wheelJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2wheelJointDefVisitor<U> {
            type Value = B2wheelJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2wheelJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let joint_def = B2wheelJointDef {
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

                    local_axis_a: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    enable_limit: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    lower_translation: seq
                        .next_element()?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                    upper_translation: seq
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
                let mut joint_def = B2wheelJointDef::default();
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
                        Field::local_axis_a => {
                            joint_def.local_axis_a = map.next_value()?;
                        }
                        Field::enable_limit => {
                            joint_def.enable_limit = map.next_value()?;
                        }
                        Field::lower_translation => {
                            joint_def.lower_translation = map.next_value()?;
                        }
                        Field::upper_translation => {
                            joint_def.upper_translation = map.next_value()?;
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

        deserializer.deserialize_struct(
            "B2wheelJointDef",
            Field::VARIANTS,
            B2wheelJointDefVisitor(self),
        )
    }
}
