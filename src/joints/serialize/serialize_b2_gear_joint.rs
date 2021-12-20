use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{ser::SerializeStruct, Deserialize, Serialize, Serializer};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::Rc;
use std::cell::RefCell;

use crate::b2_body::*;
use crate::b2_joint::*;
use crate::b2rs_common::UserDataType;

use crate::joints::b2_gear_joint::*;
use crate::serialize_b2_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

pub(crate) trait B2gearJoinToDef<D: UserDataType> {
    fn get_def(&self) -> B2gearJointDef<D>;
}

impl<D: UserDataType> B2gearJoinToDef<D> for B2gearJoint<D> {
    fn get_def(&self) -> B2gearJointDef<D> {
        return B2gearJointDef {
            base: self.base.get_def(),
            joint1: Some(self.m_joint1.clone()),
            joint2: Some(self.m_joint2.clone()),
            ratio: self.m_ratio
        };
    }
}

impl<D: UserDataType> Serialize for B2gearJointDef<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2gearJointDef", 4)?;
        state.serialize_field("base", &self.base)?;

        state.serialize_field(
            "joint1",
            &self.joint1.as_ref().unwrap().borrow().get_base().m_index,
        )?;
        state.serialize_field(
            "joint2",
            &self.joint2.as_ref().unwrap().borrow().get_base().m_index,
        )?;
        
        state.serialize_field("ratio", &self.ratio)?;

        state.end()
    }
}

pub(crate) struct B2gearJointDefContext<D: UserDataType> {
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
    pub(crate) m_all_joints: Rc<RefCell<Vec<B2jointPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2gearJointDefContext<U> {
    type Value = B2gearJointDef<U>;

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
            joint1,
            joint2,
            ratio,
        }

        struct B2gearJointDefVisitor<D: UserDataType>(B2gearJointDefContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2gearJointDefVisitor<U> {
            type Value = B2gearJointDef<U>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2gearJointDef")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let all_joints = &self.0.m_all_joints;

                let joint_def = B2gearJointDef {
                    base: seq
                        .next_element_seed(B2jointDefVisitorContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?,

                        joint1: {
                            let joint_index: i32 =  seq
                            .next_element()?
                            .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                            Some(all_joints.borrow()[joint_index as usize].clone())
                        },

                        joint2: {
                            let joint_index: i32 =  seq
                            .next_element()?
                            .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                            Some(all_joints.borrow()[joint_index as usize].clone())
                        },

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
                let mut joint_def = B2gearJointDef::default();
                let all_joints = &self.0.m_all_joints;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::base => {
                            joint_def.base = map.next_value_seed(B2jointDefVisitorContext {
                                m_body_array: self.0.m_body_array.clone(),
                            })?;
                        }
                        Field::joint1 => {
                            let joint_index: i32 = map.next_value()?;
                            joint_def.joint1 = Some(all_joints.borrow()[joint_index as usize].clone());
                        }
                        Field::joint2 => {
                            let joint_index: i32 = map.next_value()?;
                            joint_def.joint2 = Some(all_joints.borrow()[joint_index as usize].clone());
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
            "B2gearJointDef",
            Field::VARIANTS,
            B2gearJointDefVisitor(self),
        )
    }
}
