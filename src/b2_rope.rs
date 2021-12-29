use crate::b2_draw::*;
use crate::b2_math::*;

use crate::private::rope::b2_rope as private;

#[derive(Clone, Copy, PartialEq)]
pub enum B2stretchingModel {
	B2PbdStretchingModel=0,
	B2XpbdStretchingModel,
}

#[derive(Clone, Copy, PartialEq)]
pub enum B2bendingModel {
	B2SpringAngleBendingModel = 0,
	B2PbdAngleBendingModel,
	B2XpbdAngleBendingModel,
	B2PbdDistanceBendingModel,
	B2PbdHeightBendingModel,
	B2PbdTriangleBendingModel,
}

#[derive(Clone, Copy)]
pub struct B2ropeTuning {
	pub stretching_model: B2stretchingModel,
	pub bending_model: B2bendingModel,
	pub damping: f32,
	pub stretch_stiffness: f32,
	pub stretch_hertz: f32,
	pub stretch_damping: f32,
	pub bend_stiffness: f32,
	pub bend_hertz: f32,
	pub bend_damping: f32,
	pub isometric: bool,
	pub fixed_effective_mass: bool,
	pub warm_start: bool,
}

impl Default for B2ropeTuning {
	fn default() -> B2ropeTuning {
		Self {
			stretching_model: B2stretchingModel::B2PbdStretchingModel,
			bending_model: B2bendingModel::B2PbdAngleBendingModel,
			damping: 0.0,
			stretch_stiffness: 1.0,
			stretch_hertz: 0.0,
			stretch_damping: 0.0,
			bend_stiffness: 0.5,
			bend_hertz: 1.0,
			bend_damping: 0.0,
			isometric: false,
			fixed_effective_mass: false,
			warm_start: false,
		}
	}
}

#[derive(Default, Clone, Copy)]
pub struct B2ropeDefVertices {
	pub position: B2vec2,
	pub mass: f32,
}

///
pub struct B2ropeDef {
	pub position: B2vec2,
	pub vertices: Vec<B2ropeDefVertices>,
	pub gravity: B2vec2,
	pub tuning: B2ropeTuning,
}

impl Default for B2ropeDef {
	fn default() -> B2ropeDef {
		Self {
			position: B2vec2::zero(),
			vertices: Vec::default(),
			gravity: B2vec2::zero(),
			tuning: Default::default(),
		}
	}
}

#[derive(Default, Clone, Copy)]
pub(crate) struct B2ropeStretch {
	pub i1: i32,
	pub i2: i32,
	pub inv_mass1: f32,
	pub inv_mass2: f32,
	pub l: f32,
	pub lambda: f32,
	pub spring: f32,
	pub damper: f32,
}

#[derive(Default, Clone, Copy)]
pub(crate) struct B2ropeBend {
	pub i1: i32,
	pub i2: i32,
	pub i3: i32,
	pub inv_mass1: f32,
	pub inv_mass2: f32,
	pub inv_mass3: f32,
	pub inv_effective_mass: f32,
	pub lambda: f32,
	pub l1: f32,
	pub l2: f32,
	pub alpha1: f32,
	pub alpha2: f32,
	pub spring: f32,
	pub damper: f32,
}

#[derive(Default, Clone, Copy)]
pub(crate) struct B2ropePositions {
	pub(crate) m_bind_positions: B2vec2,
	pub(crate) m_ps: B2vec2,
	pub(crate) m_p0s: B2vec2,
	pub(crate) m_vs: B2vec2,
	pub(crate) m_inv_masses: f32,
}

pub struct B2rope {
	pub(crate) m_position: B2vec2,

	pub(crate) m_positions: Vec<B2ropePositions>,

	pub(crate) m_stretch_count: usize,
	pub(crate) m_bend_count: usize,

	pub(crate) m_stretch_constraints: Vec<B2ropeStretch>,
	pub(crate) m_bend_constraints: Vec<B2ropeBend>,

	pub(crate) m_gravity: B2vec2,

	pub(crate) m_tuning: B2ropeTuning,
}

impl Default for B2rope {
	fn default() -> Self {
		private::default()
	}
}

///
impl B2rope {
	///
	pub fn create(&mut self, def: &B2ropeDef) {
		private::create(self, def);
	}

	///
	pub fn set_tuning(&mut self, tuning: &B2ropeTuning) {
		private::set_tuning(self, tuning);
	}

	///
	pub fn step(&mut self, time_step: f32, iterations: i32, position: B2vec2) {
		private::step(self, time_step, iterations, position);
	}

	///
	pub fn reset(&mut self, position: B2vec2) {
		private::reset(self, position);
	}

	pub fn draw(&self, draw: &mut dyn B2drawTrait) {
		private::draw(self, draw);
	}
}
