use serde::{Deserialize, Serialize};
use serde_json;
use std::fs::File;
use std::path::Path;

const FILE_NAME: &str = "settings.ini";

impl Default for Settings {
	fn default() -> Self {
		Self {
			m_test_index: 0,
			m_window_width: 1600,
			m_window_height: 900,
			m_hertz: 60.0,
			m_velocity_iterations: 8,
			m_position_iterations: 3,
			m_draw_shapes: true,
			m_draw_joints: true,
			m_draw_aabbs: false,
			m_draw_contact_points: false,
			m_draw_contact_normals: false,
			m_draw_contact_impulse: false,
			m_draw_friction_impulse: false,
			m_draw_coms: false,
			m_draw_stats: false,
			m_draw_profile: false,
			m_enable_warm_starting: true,
			m_enable_continuous: true,
			m_enable_sub_stepping: false,
			m_enable_sleep: true,
			m_pause: false,
			m_single_step: false,
		}
	}
}

impl Settings {
	pub(crate) fn save(&self) {
		if let Ok(json_file) = File::create(FILE_NAME) {
			serde_json::to_writer_pretty(json_file, &self).expect("erro while writing json");
		}
	}
	pub(crate) fn load(&mut self) {
		let json_file_path = Path::new(FILE_NAME);
		if let Ok(json_file) = File::open(json_file_path) {
			if let Ok(result) = serde_json::from_reader(json_file){
				*self = result;
			}
		}
	}
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug)]
pub(crate) struct Settings {
	pub(crate) m_test_index: usize,
	pub(crate) m_window_width: i32,
	pub(crate) m_window_height: i32,
	pub(crate) m_hertz: f32,
	pub(crate) m_velocity_iterations: i32,
	pub(crate) m_position_iterations: i32,
	pub(crate) m_draw_shapes: bool,
	pub(crate) m_draw_joints: bool,
	pub(crate) m_draw_aabbs: bool,
	pub(crate) m_draw_contact_points: bool,
	pub(crate) m_draw_contact_normals: bool,
	pub(crate) m_draw_contact_impulse: bool,
	pub(crate) m_draw_friction_impulse: bool,
	pub(crate) m_draw_coms: bool,
	pub(crate) m_draw_stats: bool,
	pub(crate) m_draw_profile: bool,
	pub(crate) m_enable_warm_starting: bool,
	pub(crate) m_enable_continuous: bool,
	pub(crate) m_enable_sub_stepping: bool,
	pub(crate) m_enable_sleep: bool,
	pub(crate) m_pause: bool,
	pub(crate) m_single_step: bool,
}
