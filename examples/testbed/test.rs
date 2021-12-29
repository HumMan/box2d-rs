use super::draw::*;
use super::settings::*;
use super::test_private as private;
use box2d_rs::b2_body::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_contact::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_time_step::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;

use serde::{Serialize, Deserialize};

use rand::Rng;

use glium::backend::{ Facade};
use glium::glutin::event::KeyboardInput;

use std::cell::RefCell;
use std::rc::{Rc, Weak};

/// Random number in range [-1,1]
pub(crate) fn random_float() -> f32 {
	let mut rng = rand::thread_rng();
	return rng.gen_range(0.0, 10.0);
}

/// Random floating point number in range [lo, hi]
pub(crate) fn random_float_range(lo: f32, hi: f32) -> f32 {
	let mut rng = rand::thread_rng();
	return rng.gen_range(lo, hi);
}

// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
pub(crate)struct B2testDestructionListenerDefault<D: UserDataType> 
{
	pub(crate) base: TestBasePtrWeak<D>,
}
impl<D: UserDataType> B2destructionListener<D> for B2testDestructionListenerDefault<D> {
	fn say_goodbye_fixture(&mut self, fixture: FixturePtr<D>) {
		b2_not_used(fixture);
	}
	fn say_goodbye_joint(&mut self, joint: B2jointPtr<D>) {
		private::say_goodbye_joint(&mut self.base.upgrade().unwrap().borrow_mut(), joint);
	}
}

#[derive(Copy, Clone, Debug, Serialize, PartialEq, Deserialize)]
pub(crate) enum FixtureData {
	Int(i32)
}

impl Default for FixtureData
{
	fn default()->Self{
		FixtureData::Int(0)
	}
}

#[derive(Default, Copy, Clone, Debug, Serialize, Deserialize)]
pub(crate) struct UserDataTypes;
impl UserDataType for UserDataTypes {
    type Fixture = FixtureData;
    type Body = FixtureData;
    type Joint = FixtureData;
}

#[derive(Clone)]
pub(crate) struct ContactPoint<D: UserDataType> {
	pub(crate) fixture_a: FixturePtr<D>,
	pub(crate) fixture_b: FixturePtr<D>,
	pub(crate) normal: B2vec2,
	pub(crate) position: B2vec2,
	pub(crate) state: B2pointState,
	pub(crate) normal_impulse: f32,
	pub(crate) tangent_impulse: f32,
	pub(crate) separation: f32,
}

pub(crate)struct B2testContactListenerDefault<D: UserDataType> 
{
	pub(crate) base: TestBasePtrWeak<D>,
}
impl<D: UserDataType> B2contactListener<D> for B2testContactListenerDefault<D> {
	fn begin_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
		b2_not_used(contact);
	}
	fn end_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
		b2_not_used(contact);
	}
	fn pre_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, old_manifold: &B2manifold) {
		private::pre_solve(&mut self.base.upgrade().unwrap().borrow_mut(), contact, old_manifold);
	}
	fn post_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, impulse: &B2contactImpulse) {
		b2_not_used(contact);
		b2_not_used(impulse);
	}
}

pub(crate) type TestPtr<D,F> = Rc<RefCell<dyn TestDyn<D,F>>>;

pub(crate) trait TestDyn<D: UserDataType, F:Facade>
{
	fn get_base(&self) -> TestBasePtr<D>;
	
	fn step(&mut self, ui: &imgui::Ui<'_>, display: &F, target: &mut glium::Frame, settings: &mut Settings, camera: &mut Camera);
	fn update_ui(&mut self, _ui: &imgui::Ui<'_>) {}
	fn keyboard(&mut self, key: &KeyboardInput) {
		b2_not_used(key);
	}
	fn shift_mouse_down(&mut self, p: B2vec2) {
		private::shift_mouse_down(&mut self.get_base().borrow_mut(), p);
	}
	fn mouse_down(&mut self, p: B2vec2) {
		private::mouse_down(&mut self.get_base().borrow_mut(), p);
	}
	fn mouse_up(&mut self, p: B2vec2) {
		private::mouse_up(&mut self.get_base().borrow_mut(), p);
	}
	fn mouse_move(&mut self, p: B2vec2) {
		private::mouse_move(&mut self.get_base().borrow_mut(), p);
	}
	fn launch_bomb_rand(&mut self) {
		private::launch_bomb_rand(&mut self.get_base().borrow_mut());
	}
	fn launch_bomb(&mut self, position: B2vec2, velocity: B2vec2) {
		private::launch_bomb(&mut self.get_base().borrow_mut(), position, velocity);
	}
	fn spawn_bomb(&mut self, world_pt: B2vec2) {
		private::spawn_bomb(&mut self.get_base().borrow_mut(), world_pt);
	}
	fn complete_bomb_spawn(&mut self, p: B2vec2) {
		private::complete_bomb_spawn(&mut self.get_base().borrow_mut(), p);
	}

	// Let derived tests know that a joint was destroyed.
	fn joint_destroyed(&mut self, joint: B2jointPtr<D>) {
		b2_not_used(joint);
	}

	// fn shift_origin(&mut self, new_origin: B2vec2) {
	// 	private::shift_origin(&mut self.get_base().borrow_mut(), new_origin);
	// }
}

pub(crate) type TestBasePtr<D> = Rc<RefCell<Test<D>>>;
pub(crate) type TestBasePtrWeak<D> = Weak<RefCell<Test<D>>>;

#[derive(Clone)]
pub(crate) struct Test<D: UserDataType> {
	pub(crate) m_ground_body: BodyPtr<D>,
	pub(crate) m_world_aabb: B2AABB,
	pub(crate) m_points: Vec<ContactPoint<D>>,
	pub(crate) m_text_line: i32,
	pub(crate) m_world: B2worldPtr<D>,
	pub(crate) m_bomb: Option<BodyPtr<D>>,
	pub(crate) m_mouse_joint: Option<B2jointPtr<D>>,
	pub(crate) m_bomb_spawn_point: B2vec2,
	pub(crate) m_bomb_spawning: bool,
	pub(crate) m_mouse_world: B2vec2,
	pub(crate) m_step_count: i32,
	pub(crate) m_text_increment: i32,
	pub(crate) m_max_profile: B2Profile,
	pub(crate) m_total_profile: B2Profile,
	pub(crate) g_debug_draw: TestBedDebugDrawPtr,
	//pub(crate) g_camera: Camera,
}

impl<D: UserDataType> Test<D> {
	pub fn new(global_draw: TestBedDebugDrawPtr) -> Self {
		let gravity = B2vec2::new(0.0, -10.0);

		let m_world = B2world::new(gravity);

		let body_def = B2bodyDef::default();
		let m_ground_body = B2world::create_body(m_world.clone(), &body_def);

		return Self {
			m_ground_body,
			m_world_aabb: B2AABB::default(),
			m_points: Vec::new(),
			m_text_line: 0,
			m_world,
			m_bomb: None,
			m_mouse_joint: None,
			m_bomb_spawn_point: B2vec2::default(),
			m_bomb_spawning: false,
			m_mouse_world: B2vec2::default(),
			m_step_count: 0,
			m_text_increment: 13,
			m_max_profile: B2Profile::default(),
			m_total_profile: B2Profile::default(),
			g_debug_draw: global_draw,
			//g_camera: Camera::default(),
		};
	}
	pub fn draw_title(&mut self, ui: &imgui::Ui<'_>, string: &str) {
		private::draw_title(self, ui, string);
	}

	pub fn step<F:Facade>(self_: TestBasePtr<D>, ui: &imgui::Ui<'_>, display:&F, target: &mut glium::Frame, settings: &mut Settings, g_camera: Camera) {
		private::step(self_, ui, display, target, settings, g_camera);
	}
}

#[derive(Clone)]
pub(crate) struct TestEntry<'a, D: UserDataType, F:Facade> {
	pub(crate) index: usize,
	pub(crate) category: &'a str,
	pub(crate) name: &'a str,
	pub(crate) create_fcn: fn(TestBedDebugDrawPtr) -> TestPtr<D,F>,
}
