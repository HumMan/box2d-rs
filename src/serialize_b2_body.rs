use serde::de::{self, Deserializer, MapAccess, SeqAccess, Visitor};
use serde::{
    ser::{SerializeStruct},
    Deserialize, Serialize, Serializer,
};

use serde::de::DeserializeSeed;
use std::fmt;

use std::rc::{Rc};

use crate::b2_body::*;
use crate::b2_settings::*;

use crate::b2_world::*;

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
    pub(crate) m_world: B2worldWeakPtr<D>,
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
        enum Field {
            m_definition,
            m_fixture_list,
        };

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
                let world = self.0.m_world.upgrade().unwrap();

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
                let world = self.0.m_world.upgrade().unwrap();
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

pub(crate) struct B2bodyVisitorContext<D: UserDataType> {
    pub(crate) m_world: B2worldWeakPtr<D>,
}

impl<'de, U: UserDataType> DeserializeSeed<'de> for B2bodyVisitorContext<U> {
    type Value = ();

    fn deserialize<D>(self, deserializer: D) -> Result<Self::Value, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct B2bodyVisitor<D: UserDataType>(B2bodyVisitorContext<D>);

        impl<'de, U: UserDataType> Visitor<'de> for B2bodyVisitor<U> {
            type Value = ();

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct B2fixture")
            }

            fn visit_seq<V>(self, mut seq: V) -> Result<Self::Value, V::Error>
            where
                V: SeqAccess<'de>,
            {
                let context = B2bodyDefinitionVisitorContext {
                    m_world: self.0.m_world.clone(),
                };
                while let Some(elem) = seq.next_element_seed(context.clone())? {}
                Ok(())
            }
        }

        deserializer.deserialize_seq(B2bodyVisitor(self))
    }
}
