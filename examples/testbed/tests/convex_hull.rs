use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct ConvexHull<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_points: [B2vec2; B2_MAX_POLYGON_VERTICES],
	m_count: usize,
	m_auto: bool,
}

impl<D: UserDataType> ConvexHull<D> {
	const E_COUNT: usize = B2_MAX_POLYGON_VERTICES;

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
			m_points: [B2vec2::zero(); B2_MAX_POLYGON_VERTICES],
			m_count: 0,
			m_auto: false,
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
		self.generate();
		self.m_auto = false;
	}

	fn generate(&mut self) {
		let lower_bound = B2vec2::new(-8.0, -8.0);
		let upper_bound = B2vec2::new(8.0, 8.0);

		for i in 0..Self::E_COUNT {
			let x: f32 = 10.0 * random_float();
			let y: f32 = 10.0 * random_float();

			// Clamp onto a square to help create collinearities.
			// This will stress the convex hull algorithm.
			let mut v = B2vec2::new(x, y);
			v = b2_clamp_vec2(v, lower_bound, upper_bound);
			self.m_points[i] = v;
		}

		self.m_count = Self::E_COUNT;
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for ConvexHull<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}

	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state != ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::A) => {
					self.m_auto = !self.m_auto;
				}
				Some(VirtualKeyCode::G) => {
					self.generate();
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

		{
			let mut shape = B2polygonShape::default();
			shape.set(&self.m_points[..self.m_count]);

			let mut base = self.base.borrow_mut();

			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Press g to generate a new random convex hull",
			);
			base.m_text_line += base.m_text_increment;

			base.g_debug_draw.borrow_mut().draw_polygon(
				&shape.m_vertices[..shape.m_count],
				B2color::new(0.9, 0.9, 0.9),
			);

			for i in 0..self.m_count {
				base.g_debug_draw.borrow_mut().draw_point(
					self.m_points[i],
					3.0,
					B2color::new(0.3, 0.9, 0.3),
				);
				base.g_debug_draw.borrow_mut().draw_string(
					ui,
					self.m_points[i] + B2vec2::new(0.05, 0.05),
					&format!("{0}", i),
				);
			}

			if shape.validate() == false {
				base.m_text_line += 0;
			}
		}

		if self.m_auto {
			self.generate();
		}
	}
}
