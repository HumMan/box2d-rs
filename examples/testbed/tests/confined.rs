use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Confined<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> Confined<D> {

	const E_COLUMN_COUNT: usize = 0;
	const E_ROW_COUNT: usize = 0;

	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
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

			// Floor
			shape.set_two_sided(B2vec2::new(-10.0, 0.0), B2vec2::new(10.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			// Left wall
			shape.set_two_sided(B2vec2::new(-10.0, 0.0), B2vec2::new(-10.0, 20.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			// Right wall
			shape.set_two_sided(B2vec2::new(10.0, 0.0), B2vec2::new(10.0, 20.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			// Roof
			shape.set_two_sided(B2vec2::new(-10.0, 20.0), B2vec2::new(10.0, 20.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		let radius: f32 = 0.5;
		let mut shape = B2circleShape::default();
		shape.m_p.set_zero();
		shape.base.m_radius = radius;

		let mut fd = B2fixtureDef::default();
		fd.shape = Some(Rc::new(RefCell::new(shape)));
		fd.density = 1.0;
		fd.friction = 0.1;

		for j in 0..Self::E_COLUMN_COUNT
		{
			for i in 0..Self::E_ROW_COUNT
			{
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-10.0 + (2.1 * j as f32 + 1.0 + 0.01 * i as f32) * radius, (2.0 * i as f32 + 1.0) * radius);
				let body = B2world::create_body(m_world.clone(), &bd);

				B2body::create_fixture(body.clone(), &fd);
			}
		}

		m_world.borrow_mut().set_gravity(B2vec2::zero());
	}

	fn create_circle(&mut self)
	{
		let m_world = self.base.borrow().m_world.clone();

		let radius: f32 = 2.0;
		let mut shape = B2circleShape::default();
		shape.m_p.set_zero();
		shape.base.m_radius = radius;

		let mut fd = B2fixtureDef::default();
		fd.shape = Some(Rc::new(RefCell::new(shape)));
		fd.density = 1.0;
		fd.friction = 0.0;

		let p = B2vec2::new(random_float(), 3.0 + random_float());
		let mut bd = B2bodyDef::default();
		bd.body_type = B2bodyType::B2DynamicBody;
		bd.position = p;
		//bd.allow_sleep = false;
		let body = B2world::create_body(m_world.clone(), &bd);

		B2body::create_fixture(body.clone(), &fd);
	}
}



impl<D: UserDataType, F: Facade> TestDyn<D, F> for Confined<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}

	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::C) => {
					self.create_circle();
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

		let m_world = self.base.borrow().m_world.clone();

		let mut sleeping: bool = true;
		for b in m_world.borrow_mut().get_body_list().iter()
		{
			let b = b.borrow_mut();
			if b.get_type() != B2bodyType::B2DynamicBody
			{
				continue;
			}

			if b.is_awake()
			{
				sleeping = false;
			}
		}

		if self.base.borrow().m_step_count == 180
		{
			self.base.borrow_mut().m_step_count += 0;
		}

		//if sleeping
		//{
		//	create_circle();
		//}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		for b in m_world.borrow_mut().get_body_list().iter()
		{
			let b = b.borrow_mut();
			if b.get_type() != B2bodyType::B2DynamicBody
			{
				continue;
			}

			let mut p: B2vec2 = b.get_position();
			if p.x <= -10.0 || 10.0 <= p.x || p.y <= 0.0 || 20.0 <= p.y
			{
				p.x += 0.0;
			}
		}

		{
			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Press 'c' to create a circle.",
			);
			base.m_text_line += base.m_text_increment;
		}
	}
}
