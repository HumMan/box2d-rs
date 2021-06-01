use serde::de::{self, Deserializer, Error, MapAccess, SeqAccess, Visitor};
use serde::{
    ser::{SerializeSeq, SerializeStruct},
    Deserialize, Serialize, Serializer,
};
use std::fmt;
use std::marker::PhantomData;
use std::rc::Rc;

use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_world::*;

use crate::joints::serialize::serialize_b2_distance_joint::*;
use crate::joints::serialize::serialize_b2_revolute_joint::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

use crate::double_linked_list::*;
use crate::serialize_b2_body::*;

struct JointWithType<T: Serialize> {
    jtype: B2jointType,
    joint_def: T,
}

impl<T> Serialize for JointWithType<T>
where
    T: Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("JointWithType", 2)?;
        state.serialize_field("jtype", &self.jtype)?;
        state.serialize_field("joint_def", &self.joint_def)?;
        state.end()
    }
}

impl<D> Serialize for DoubleLinkedList<dyn B2jointTraitDyn<D>>
where
    D: UserDataType,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let len = self.iter().count();
        let mut state = serializer.serialize_seq(Some(len))?;
        for j in self.iter() {
            match j.borrow().as_derived() {
                JointAsDerived::EDistanceJoint(joint) => {
                    let def = joint.get_def();
                    state.serialize_element(&JointWithType {
                        jtype: def.base.jtype,
                        joint_def: def,
                    })?;
                }
                JointAsDerived::EFrictionJoint(joint) => {}
                JointAsDerived::EGearJoint(joint) => {}
                JointAsDerived::EMouseJoint(joint) => {}
                JointAsDerived::EMotorJoint(joint) => {}
                JointAsDerived::EPulleyJoint(joint) => {}
                JointAsDerived::ERevoluteJoint(joint) => {
                    let def = joint.get_def();
                    state.serialize_element(&JointWithType {
                        jtype: def.base.jtype,
                        joint_def: def,
                    })?;
                }
                JointAsDerived::ERopeJoint(joint) => {}
                JointAsDerived::EPrismaticJoint(joint) => {}
                JointAsDerived::EWeldJoint(joint) => {}
                JointAsDerived::EWheelJoint(joint) => {}
            }
        }
        state.end()
    }
}

impl<D: UserDataType> Serialize for B2world<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2world", 2)?;

        state.serialize_field("m_gravity", &self.m_gravity)?;
        state.serialize_field("m_bodies_list", &self.m_body_list)?;
        for (i, ref mut b) in self.m_body_list.iter().enumerate() {
            b.borrow_mut().m_island_index = i as i32;
        }

        state.serialize_field("m_joints_list", &self.m_joint_list)?;

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
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        #[derive(EnumVariantNames)]
        #[allow(non_camel_case_types)]
        enum Field {
            m_gravity,
            m_bodies_list,
            m_joints_list
        }
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
                let world = B2world::new(m_gravity);

                seq.next_element_seed(B2bodyListContext {
                    m_world: Rc::downgrade(&world),
                })?
                .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                let m_body_array;
                {
                    m_body_array = world.borrow().m_body_list.iter().collect::<Vec<_>>();
                }  

                seq.next_element_seed(B2jointListContext {
                                m_world: Rc::downgrade(&world.clone()),
                                m_body_array
                            })?
                .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                
                Ok(world)
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut world = None;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::m_gravity => {
                            let m_gravity = map.next_value()?;
                            world = Some(B2world::<U>::new(m_gravity));
                        }
                        Field::m_bodies_list => {                            
                            map.next_value_seed(B2bodyListContext {
                                m_world: Rc::downgrade(&world.as_ref().unwrap()),
                            })?;
                        }
                        Field::m_joints_list => {                          
                            let m_body_array;
                            {
                                m_body_array = world.as_ref().unwrap().borrow().m_body_list.iter().collect::<Vec<_>>();
                            }  
                            map.next_value_seed(B2jointListContext {
                                m_world: Rc::downgrade(&world.as_ref().unwrap()),
                                m_body_array
                            })?;
                        }
                    }
                }
                Ok(world.unwrap())
            }
        }

        let result =
            deserializer.deserialize_struct("B2world", Field::VARIANTS, B2worldVisitor::default());
        Ok(B2worldDeserializeResult { world: result? })
    }
}
