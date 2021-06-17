use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{
    ser::{SerializeStruct},
    Deserialize, Serialize, Serializer,
};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::{Rc};
use std::cell::RefCell;

use crate::b2_body::*;
use crate::b2_settings::*;
use crate::b2_joint::*;
use crate::b2_world::*;

use crate::joints::serialize::serialize_b2_distance_joint::*;
use crate::joints::serialize::serialize_b2_friction_joint::*;
use crate::joints::serialize::serialize_b2_gear_joint::*;
use crate::joints::serialize::serialize_b2_motor_joint::*;
use crate::joints::serialize::serialize_b2_prismatic_joint::*;
use crate::joints::serialize::serialize_b2_pulley_joint::*;
use crate::joints::serialize::serialize_b2_revolute_joint::*;
use crate::joints::serialize::serialize_b2_rope_joint::*;
use crate::joints::serialize::serialize_b2_weld_joint::*;
use crate::joints::serialize::serialize_b2_wheel_joint::*;

use crate::serialize_b2_fixture::*;

use strum::VariantNames;
use strum_macros::EnumVariantNames;

trait B2bodyToDef<D: UserDataType> {
    fn get_def(&self) -> B2bodyDef<D>;
}

impl<D: UserDataType> B2bodyToDef<D> for B2body<D> {
    fn get_def(&self) -> B2bodyDef<D> {
        return B2bodyDef {
            body_type: self.m_type,
            position: self.m_xf.p,
            angle: self.m_sweep.a,
            linear_velocity: self.m_linear_velocity,
            angular_velocity: self.m_angular_velocity,
            linear_damping: self.m_linear_damping,
            angular_damping: self.m_angular_damping,
            allow_sleep: self.m_flags.contains(BodyFlags::E_AUTO_SLEEP_FLAG),
            awake: self.m_flags.contains(BodyFlags::E_AWAKE_FLAG),
            fixed_rotation: self.m_flags.contains(BodyFlags::E_FIXED_ROTATION_FLAG),
            bullet: self.m_flags.contains(BodyFlags::E_BULLET_FLAG),
            enabled: self.m_flags.contains(BodyFlags::E_ENABLED_FLAG),
            gravity_scale: self.m_gravity_scale,
            user_data: self.m_user_data.clone(),
        };
    }
}

impl<D: UserDataType> Serialize for B2body<D> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("B2body", 2)?;

        let definition = self.get_def();
        state.serialize_field("m_definition", &definition)?;
        state.serialize_field("m_fixture_list", &self.m_fixture_list)?;
        state.end()
    }
}

#[derive(Clone)]
pub(crate) struct B2bodyDefinitionVisitorContext<D: UserDataType> {
    pub(crate) m_world: B2worldPtr<D>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2bodyDefinitionVisitorContext<U> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        #[derive(EnumVariantNames)]
        #[allow(non_camel_case_types)]
        enum Field {
            m_definition,
            m_fixture_list,
        }

        struct B2bodyDefinitionVisitor<D: UserDataType>(B2bodyDefinitionVisitorContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2bodyDefinitionVisitor<U> {
            type Value = ();

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2fixture")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let world = self.0.m_world.clone();

                let def: B2bodyDef<U> = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;
                let body = Some(B2world::create_body(world.clone(), &def));

                seq.next_element_seed(B2fixtureListVisitorContext {
                    body: Rc::downgrade(&body.clone().unwrap()),
                })?
                .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                Ok(())
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut body = None;
                let world = self.0.m_world;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::m_definition => {
                            let def: B2bodyDef<U> = map.next_value()?;
                            body = Some(B2world::create_body(world.clone(), &def));
                        }
                        Field::m_fixture_list => {
                            map.next_value_seed(B2fixtureListVisitorContext {
                                body: Rc::downgrade(&body.clone().unwrap()),
                            })?;
                        }
                    }
                }
                Ok(())
            }
        }

        deserializer.deserialize_struct("B2body", Field::VARIANTS, B2bodyDefinitionVisitor(self))
    }
}

pub(crate) struct B2bodyListContext<D: UserDataType> {
    pub(crate) m_world: B2worldPtr<D>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2bodyListContext<U> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct B2bodyVisitor<D: UserDataType>(B2bodyListContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2bodyVisitor<U> {
            type Value = ();

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2bodyListContext")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let context = B2bodyDefinitionVisitorContext {
                    m_world: self.0.m_world.clone(),
                };
                while let Some(_elem) = seq.next_element_seed(context.clone())? {}
                Ok(())
            }
        }

        deserializer.deserialize_seq(B2bodyVisitor(self))
    }
}


#[derive(Clone)]
pub(crate) struct B2jointDefinitionVisitorContext<D: UserDataType> {
    pub(crate) m_world: B2worldPtr<D>,
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
    pub(crate) m_all_joints: Rc<RefCell<Vec<B2jointPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2jointDefinitionVisitorContext<U> {
    type Value = ();

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
            joint_def,
        }

        struct B2jointDefinitionVisitor<D: UserDataType>(B2jointDefinitionVisitorContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2jointDefinitionVisitor<U> {
            type Value = ();

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2fixture")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let world = self.0.m_world.clone();

                let jtype: B2jointType = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                match jtype {
                    B2jointType::EUnknownJoint =>{
                        panic!();
                    }
                    B2jointType::EDistanceJoint => {
                        let def = seq.next_element_seed(B2distanceJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::DistanceJoint(def));
                    }
                    B2jointType::EFrictionJoint => {
                        let def = seq.next_element_seed(B2frictionJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::FrictionJoint(def));
                    }
                    B2jointType::EGearJoint => {
                        let all_joints = self.0.m_all_joints.clone();
                        let def = seq.next_element_seed(B2gearJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                            m_all_joints: all_joints.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::GearJoint(def));
                    }
                    B2jointType::EMouseJoint => {
                        panic!();
                    }
                    B2jointType::EMotorJoint => {
                        let def = seq.next_element_seed(B2motorJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::MotorJoint(def));
                    }
                    B2jointType::EPulleyJoint => {
                        let def = seq.next_element_seed(B2pulleyJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::PulleyJoint(def));
                    }
                    B2jointType::ERevoluteJoint => {
                        let def = seq.next_element_seed(B2revoluteJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::RevoluteJoint(def));
                    }
                    B2jointType::ERopeJoint => {
                        let def = seq.next_element_seed(B2ropeJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::RopeJoint(def));
                    }
                    B2jointType::EPrismaticJoint => {
                        let def = seq.next_element_seed(B2prismaticJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::PrismaticJoint(def));
                    }
                    B2jointType::EWeldJoint => {
                        let def = seq.next_element_seed(B2weldJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::WeldJoint(def));
                    }
                    B2jointType::EWheelJoint => {
                        let def = seq.next_element_seed(B2wheelJointDefContext {
                            m_body_array: self.0.m_body_array.clone(),
                        })?
                        .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                        world.borrow_mut().create_joint(&B2JointDefEnum::WheelJoint(def));
                    }
                }

                Ok(())
            }

            fn visit_map<V>(self, mut map: V) -> Result<Self::Value, V::Error>
            where
                V: MapAccess<'de>,
            {
                let world = self.0.m_world;
                let mut jtype: Option<B2jointType> = None;
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::jtype => {
                            jtype = Some(map.next_value()?);
                        }
                        Field::joint_def => {
                            match jtype.unwrap() {
                                B2jointType::EUnknownJoint =>{
                                    panic!();
                                }
                                B2jointType::EDistanceJoint => {
                                    let def = map.next_value_seed(B2distanceJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::DistanceJoint(def));
                                }
                                B2jointType::EFrictionJoint => {
                                    let def = map.next_value_seed(B2frictionJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::FrictionJoint(def));
                                }
                                B2jointType::EGearJoint => {
                                    let all_joints = self.0.m_all_joints.clone();
                                    let def = map.next_value_seed(B2gearJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                        m_all_joints: all_joints.clone(),
                                    })?;

                                    let gear_joint = world.borrow_mut().create_joint(&B2JointDefEnum::GearJoint(def));
                                    all_joints.borrow_mut().push(gear_joint);
                                }
                                B2jointType::EMouseJoint => {}
                                B2jointType::EMotorJoint => {
                                    let def = map.next_value_seed(B2motorJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::MotorJoint(def));
                                }
                                B2jointType::EPulleyJoint => {
                                    let def = map.next_value_seed(B2pulleyJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::PulleyJoint(def));
                                }
                                B2jointType::ERevoluteJoint => {
                                    let def = map.next_value_seed(B2revoluteJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::RevoluteJoint(def));
                                }
                                B2jointType::ERopeJoint => {
                                    let def = map.next_value_seed(B2ropeJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::RopeJoint(def));
                                }
                                B2jointType::EPrismaticJoint => {
                                    let def = map.next_value_seed(B2prismaticJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::PrismaticJoint(def));
                                }
                                B2jointType::EWeldJoint => {
                                    let def = map.next_value_seed(B2weldJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::WeldJoint(def));
                                }
                                B2jointType::EWheelJoint => {
                                    let def = map.next_value_seed(B2wheelJointDefContext {
                                        m_body_array: self.0.m_body_array.clone(),
                                    })?;
                                    world.borrow_mut().create_joint(&B2JointDefEnum::WheelJoint(def));
                                }
                            }
                            
                        }
                    }
                }
                
                Ok(())
            }
        }

        deserializer.deserialize_struct("B2body", Field::VARIANTS, B2jointDefinitionVisitor(self))
    }
}


pub(crate) struct B2jointListContext<D: UserDataType> {
    pub(crate) m_world: B2worldPtr<D>,
    pub(crate) m_body_array: Rc<RefCell<Vec<BodyPtr<D>>>>,
    pub(crate) m_all_joints: Rc<RefCell<Vec<B2jointPtr<D>>>>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2jointListContext<U> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct B2jointListVisitor<D: UserDataType>(B2jointListContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2jointListVisitor<U> {
            type Value = ();

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2jointListContext")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let context = B2jointDefinitionVisitorContext {
                    m_world: self.0.m_world.clone(),
                    m_body_array: self.0.m_body_array.clone(),
                    m_all_joints: self.0.m_all_joints.clone(),
                };
                while let Some(_elem) = seq.next_element_seed(context.clone())? {}
                Ok(())
            }
        }

        deserializer.deserialize_seq(B2jointListVisitor(self))
    }
}