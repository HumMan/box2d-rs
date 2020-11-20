use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct ShapeEditing<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
	m_body: Option<BodyPtr<D>>,
	m_fixture1: Option<FixturePtr<D>>,
	m_fixture2: Option<FixturePtr<D>>,
	m_sensor: bool,
}

impl ShapeEditing<UserDataTypes> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<UserDataTypes, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),

			m_body: None,
			m_fixture1: None,
			m_fixture2: None,
			m_sensor: false,
		}));

		{
			let mut self_ = result_ptr.borrow_mut();
			{
				let world = base.borrow().m_world.clone();
				let mut world = world.borrow_mut();
				world.set_destruction_listener(self_.destruction_listener.clone());
				world.set_contact_listener(self_.contact_listener.clone());
				world.set_debug_draw(global_draw);
			}
			self_.init();
		}

		return result_ptr;
	}
	fn init(&mut self) {
		let m_world = self.base.borrow().m_world.clone();
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		let mut bd = B2bodyDef::default();
		bd.body_type = B2bodyType::B2DynamicBody;
		bd.position.set(0.0, 10.0);
		let m_body = B2world::create_body(m_world.clone(), &bd);
		self.m_body = Some(m_body.clone());

		let mut shape = B2polygonShape::default();
		shape.set_as_box_angle(4.0, 4.0, B2vec2::zero(), 0.0);
		self.m_fixture1 = Some(B2body::create_fixture_by_shape(
			m_body,
			Rc::new(RefCell::new(shape)),
			10.0,
		));

		self.m_fixture2 = None;

		self.m_sensor = false;
	}
}

impl<F: Facade> TestDyn<UserDataTypes, F> for ShapeEditing<UserDataTypes> {
	fn get_base(&self) -> TestBasePtr<UserDataTypes> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::C) => {
					if self.m_fixture2.is_none() {
						let mut shape = B2circleShape::default();
						shape.base.m_radius = 3.0;
						shape.m_p.set(0.5, -4.0);
						let m_body = self.m_body.clone().unwrap();
						self.m_fixture2 = Some(B2body::create_fixture_by_shape(
							m_body.clone(),
							Rc::new(RefCell::new(shape)),
							10.0,
						));
						m_body.borrow_mut().set_awake(true);
					}
				}
				Some(VirtualKeyCode::D) => {
					if let Some(m_fixture2) = self.m_fixture2.take() {
						let m_body = self.m_body.clone().unwrap();
						B2body::destroy_fixture(m_body.clone(), m_fixture2);
						m_body.borrow_mut().set_awake(true);
					}
				}
				Some(VirtualKeyCode::S) => {
					if let Some(m_fixture2) = self.m_fixture2.clone() {
						self.m_sensor = !self.m_sensor;
						m_fixture2.borrow_mut().set_sensor(self.m_sensor);
					}
				}
				_ => (),
			}
		}
	}
	fn step(
		&mut self,
		ui: &imgui::Ui<'_>,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		let mut base = self.base.borrow_mut();
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press: (c) create a shape, (d) destroy a shape.",
		);
		base.m_text_line += base.m_text_increment;

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("sensor = {0}", self.m_sensor),
		);
		base.m_text_line += base.m_text_increment;
	}
}

//static int testIndex = RegisterTest("Examples", "Shape Editing", ShapeEditing::create);
