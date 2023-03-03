use super::super::draw::*;
use super::super::settings::*;
use box2d_rs::b2rs_common::UserDataType;
use super::super::test::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use box2d_rs::b2_distance::*;
use box2d_rs::b2_draw::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct DistanceTest<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_position_b: B2vec2,
	m_angle_b: f32,

	m_transform_a: B2Transform,
	m_transform_b: B2Transform,
	m_polygon_a: B2polygonShape,
	m_polygon_b: B2polygonShape,
}

impl<D: UserDataType> DistanceTest<D> {
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
			m_position_b: B2vec2::zero(),
			m_angle_b: 0.0,

			m_transform_a: B2Transform::default(),
			m_transform_b: B2Transform::default(),
			m_polygon_a: B2polygonShape::default(),
			m_polygon_b: B2polygonShape::default(),
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
		{
			self.m_transform_a.set_identity();
			self.m_transform_a.p.set(0.0, -0.2);
			self.m_polygon_a.set_as_box(10.0, 0.2);
		}

		{
			self.m_position_b.set(12.017401, 0.13678508);
			self.m_angle_b = -0.0109265;
			self.m_transform_b.set(self.m_position_b, self.m_angle_b);

			self.m_polygon_b.set_as_box(2.0, 0.1);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for DistanceTest<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state != ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::A) => {
					self.m_position_b.x -= 0.1;
				}
				Some(VirtualKeyCode::D) => {
					self.m_position_b.x += 0.1;
				}
				Some(VirtualKeyCode::S) => {
					self.m_position_b.y -= 0.1;
				}
				Some(VirtualKeyCode::W) => {
					self.m_position_b.y += 0.1;
				}
				Some(VirtualKeyCode::Q) => {
					self.m_angle_b += 0.1 * B2_PI;
				}
				Some(VirtualKeyCode::E) => {
					self.m_angle_b -= 0.1 * B2_PI;
				}
				_ => (),
			}

			self.m_transform_b.set(self.m_position_b, self.m_angle_b);
		}
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

		let mut input = B2distanceInput::default();
		input.proxy_a.set_shape(Rc::new(self.m_polygon_a), 0);
		input.proxy_b.set_shape(Rc::new(self.m_polygon_b), 0);
		input.transform_a = self.m_transform_a;
		input.transform_b = self.m_transform_b;
		input.use_radii = true;
		let mut cache = B2simplexCache::default();
		cache.count = 0;
		let mut output = B2distanceOutput::default();
		b2_distance_fn(&mut output, &mut cache, &input);

		let mut base = self.base.borrow_mut();
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("distance = {0}", output.distance),
		);
		base.m_text_line += base.m_text_increment;

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!("iterations = {0}", output.iterations),
		);
		base.m_text_line += base.m_text_increment;
		{
			let color = B2color::new(0.9, 0.9, 0.9);
			let mut v: [B2vec2; B2_MAX_POLYGON_VERTICES] =
				[B2vec2::zero(); B2_MAX_POLYGON_VERTICES];
			for i in 0..self.m_polygon_a.m_count {
				v[i] = b2_mul_transform_by_vec2(self.m_transform_a, self.m_polygon_a.m_vertices[i]);
			}
			base.g_debug_draw
				.borrow_mut()
				.draw_polygon(&v[..self.m_polygon_a.m_count], color);

			for i in 0..self.m_polygon_b.m_count {
				v[i] = b2_mul_transform_by_vec2(self.m_transform_b, self.m_polygon_b.m_vertices[i]);
			}
			base.g_debug_draw
				.borrow_mut()
				.draw_polygon(&v[..self.m_polygon_b.m_count], color);
		}

		let x1: B2vec2 = output.point_a;
		let x2: B2vec2 = output.point_b;

		let c1 = B2color::new(1.0, 0.0, 0.0);
		base.g_debug_draw.borrow_mut().draw_point(x1, 4.0, c1);

		let c2 = B2color::new(1.0, 1.0, 0.0);
		base.g_debug_draw.borrow_mut().draw_point(x2, 4.0, c2);
	}
}
