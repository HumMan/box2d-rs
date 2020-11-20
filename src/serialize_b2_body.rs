use std::cell::RefCell;
use std::rc::{Rc, Weak};
use serde::{Serialize, Deserialize, Serializer, ser::{SerializeStruct, SerializeSeq}};

use crate::b2_contact::*;
use crate::b2_fixture::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_settings::*;
use crate::b2_shape::*;
use crate::b2_world::*;
use crate::private::dynamics::b2_body as private;

use crate::linked_list::*;
use crate::double_linked_list::*;

// impl<D: UserDataType> Serialize for B2body<D>
// {
//     fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
//     where
//         S: Serializer,
//     {
// 		let mut state = serializer.serialize_struct("B2body", 19)?;
// 		state.serialize_field("m_type", &self.m_type)?;

// 		state.serialize_field("m_flags", &self.m_flags)?;

// 		state.serialize_field("m_island_index", &self.m_island_index)?;

// 		state.serialize_field("m_xf", &self.m_xf)?;
// 		state.serialize_field("m_sweep", &self.m_sweep)?;

// 		state.serialize_field("m_linear_velocity", &self.m_linear_velocity)?;
// 		state.serialize_field("m_angular_velocity", &self.m_angular_velocity)?;

// 		state.serialize_field("m_force", &self.m_force)?;
// 		state.serialize_field("m_torque", &self.m_torque)?;

// 		state.serialize_field("m_fixture_count", &self.m_fixture_count)?;

// 		state.serialize_field("m_mass", &self.m_mass)?;
// 		state.serialize_field("m_inv_mass", &self.m_inv_mass)?;

// 		state.serialize_field("m_i", &self.m_i)?;
// 		state.serialize_field("m_inv_i", &self.m_inv_i)?;

// 		state.serialize_field("m_linear_damping", &self.m_linear_damping)?;
// 		state.serialize_field("m_angular_damping", &self.m_angular_damping)?;
// 		state.serialize_field("m_gravity_scale", &self.m_gravity_scale)?;

// 		state.serialize_field("m_sleep_time", &self.m_sleep_time)?;

// 		// {
// 		// 	let mut state = serializer.serialize_struct("B2body", 19)?;
// 		// 	state.serialize_field("m_type", &self.m_type)?;
// 		// }
//         state.end()
//     }
// }