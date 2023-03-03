use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;

use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct EdgeTest<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_offset1: B2vec2,
	m_offset2: B2vec2,
	m_body1: Option<BodyPtr<D>>,
	m_body2: Option<BodyPtr<D>>,
	m_boxes: bool,
}

impl<D: UserDataType> EdgeTest<D> {
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

			m_offset1: B2vec2::new(0.0, 8.0),
			m_offset2: B2vec2::new(0.0, 16.0),
			m_body1: None,
			m_body2: None,
			m_boxes: true,
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

		let vertices: [B2vec2; 10] = [
			B2vec2::new(10.0, -4.0),
			B2vec2::new(10.0, 0.0),
			B2vec2::new(6.0, 0.0),
			B2vec2::new(4.0, 2.0),
			B2vec2::new(2.0, 0.0),
			B2vec2::new(-2.0, 0.0),
			B2vec2::new(-6.0, 0.0),
			B2vec2::new(-8.0, -3.0),
			B2vec2::new(-10.0, 0.0),
			B2vec2::new(-10.0, -4.0),
		];

		{
			let v1: B2vec2 = vertices[0] + self.m_offset1;
			let v2: B2vec2 = vertices[1] + self.m_offset1;
			let v3: B2vec2 = vertices[2] + self.m_offset1;
			let v4: B2vec2 = vertices[3] + self.m_offset1;
			let v5: B2vec2 = vertices[4] + self.m_offset1;
			let v6: B2vec2 = vertices[5] + self.m_offset1;
			let v7: B2vec2 = vertices[6] + self.m_offset1;
			let v8: B2vec2 = vertices[7] + self.m_offset1;
			let v9: B2vec2 = vertices[8] + self.m_offset1;
			let v10: B2vec2 = vertices[9] + self.m_offset1;

			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();

			shape.set_one_sided(v10, v1, v2, v3);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v1, v2, v3, v4);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v2, v3, v4, v5);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v3, v4, v5, v6);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v4, v5, v6, v7);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v5, v6, v7, v8);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v6, v7, v8, v9);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v7, v8, v9, v10);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v8, v9, v10, v1);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_one_sided(v9, v10, v1, v2);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let v1: B2vec2 = vertices[0] + self.m_offset2;
			let v2: B2vec2 = vertices[1] + self.m_offset2;
			let v3: B2vec2 = vertices[2] + self.m_offset2;
			let v4: B2vec2 = vertices[3] + self.m_offset2;
			let v5: B2vec2 = vertices[4] + self.m_offset2;
			let v6: B2vec2 = vertices[5] + self.m_offset2;
			let v7: B2vec2 = vertices[6] + self.m_offset2;
			let v8: B2vec2 = vertices[7] + self.m_offset2;
			let v9: B2vec2 = vertices[8] + self.m_offset2;
			let v10: B2vec2 = vertices[9] + self.m_offset2;

			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();

			shape.set_two_sided(v1, v2);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v2, v3);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v3, v4);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v4, v5);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v5, v6);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v6, v7);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v7, v8);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v8, v9);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v9, v10);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

			shape.set_two_sided(v10, v1);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		self.create_boxes();
	}

	fn create_boxes(&mut self) {
		let m_world = self.base.borrow().m_world.clone();
		if let Some(m_body1) = self.m_body1.take() {
			m_world.borrow_mut().destroy_body(m_body1);
		}

		if let Some(m_body2) = self.m_body2.take() {
			m_world.borrow_mut().destroy_body(m_body2);
		}

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position = B2vec2::new(8.0, 2.6) + self.m_offset1;
			bd.allow_sleep = false;
			let m_body1 = B2world::create_body(m_world.clone(), &bd);
			self.m_body1 = Some(m_body1.clone());

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 1.0);

			B2body::create_fixture_by_shape(m_body1.clone(), Rc::new(RefCell::new(shape)), 1.0);
		}

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position = B2vec2::new(8.0, 2.6) + self.m_offset2;
			bd.allow_sleep = false;
			let m_body2 = B2world::create_body(m_world.clone(), &bd);
			self.m_body2 = Some(m_body2.clone());

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 1.0);

			B2body::create_fixture_by_shape(m_body2.clone(), Rc::new(RefCell::new(shape)), 1.0);
		}
	}

	fn create_circles(&mut self) {
		let m_world = self.base.borrow().m_world.clone();
		if let Some(m_body1) = self.m_body1.take() {
			m_world.borrow_mut().destroy_body(m_body1);
		}

		if let Some(m_body2) = self.m_body2.take() {
			m_world.borrow_mut().destroy_body(m_body2);
		}

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position = B2vec2::new(-0.5, 0.6) + self.m_offset1;
			bd.allow_sleep = false;
			let m_body1 = B2world::create_body(m_world.clone(), &bd);
			self.m_body1 = Some(m_body1.clone());

			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.5;

			B2body::create_fixture_by_shape(m_body1.clone(), Rc::new(RefCell::new(shape)), 1.0);
		}

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position = B2vec2::new(-0.5, 0.6) + self.m_offset2;
			bd.allow_sleep = false;
			let m_body2 = B2world::create_body(m_world.clone(), &bd);
			self.m_body2 = Some(m_body2.clone());

			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.5;

			B2body::create_fixture_by_shape(m_body2.clone(), Rc::new(RefCell::new(shape)), 1.0);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for EdgeTest<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::A) => {
					self.m_body1
						.clone()
						.unwrap()
						.borrow_mut()
						.apply_force_to_center(B2vec2::new(-10.0, 0.0), true);
					self.m_body2
						.clone()
						.unwrap()
						.borrow_mut()
						.apply_force_to_center(B2vec2::new(-10.0, 0.0), true);
				}
				Some(VirtualKeyCode::D) => {
					self.m_body1
						.clone()
						.unwrap()
						.borrow_mut()
						.apply_force_to_center(B2vec2::new(10.0, 0.0), true);
					self.m_body2
						.clone()
						.unwrap()
						.borrow_mut()
						.apply_force_to_center(B2vec2::new(10.0, 0.0), true);
				}
				_ => (),
			}
		}
	}
	fn update_ui(&mut self, ui: &imgui::Ui) {
		ui.window("Custom Controls")
			.flags(
				imgui::WindowFlags::NO_MOVE
					| imgui::WindowFlags::NO_RESIZE
					| imgui::WindowFlags::NO_COLLAPSE,
			)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 100.0], imgui::Condition::Always)
			.build(|| {
				if ui.radio_button_bool("Boxes", self.m_boxes == true) {
					self.create_boxes();
					self.m_boxes = true;
				}

				if ui.radio_button_bool("Circles", self.m_boxes == false) {
					self.create_circles();
					self.m_boxes = false;
				}
			});
	}

	fn step(
		&mut self,
		ui: &imgui::Ui,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}
