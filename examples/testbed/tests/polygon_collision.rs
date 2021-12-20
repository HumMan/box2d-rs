use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct PolygonCollision<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_polygon_a: B2polygonShape,
	m_polygon_b: B2polygonShape,

	m_transform_a: B2Transform,
	m_transform_b: B2Transform,

	m_position_b: B2vec2,
	m_angle_b: f32,
}

impl<D: UserDataType> PolygonCollision<D> {
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
			m_polygon_a: B2polygonShape::default(),
			m_polygon_b: B2polygonShape::default(),
			m_transform_a: B2Transform::default(),
			m_transform_b: B2Transform::default(),
			m_position_b: B2vec2::default(),
			m_angle_b: 0.0,
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
			self.m_polygon_a.set_as_box(0.2, 0.4);
			self.m_transform_a.set(B2vec2::zero(), 0.0);
		}

		{
			self.m_polygon_b.set_as_box(0.5, 0.5);
			self.m_position_b.set(19.345284, 1.5632932);
			self.m_angle_b = 1.9160721;
			self.m_transform_b.set(self.m_position_b, self.m_angle_b);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for PolygonCollision<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
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
		}

		self.m_transform_b.set(self.m_position_b, self.m_angle_b);
	}
	fn step(
		&mut self,
		ui: &imgui::Ui<'_>,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		{
			let mut manifold = B2manifold::default();
			b2_collide_polygons(
				&mut manifold,
				&self.m_polygon_a,
				&self.m_transform_a,
				&self.m_polygon_b,
				&self.m_transform_b,
			);

			let mut world_manifold = B2worldManifold::default();
			world_manifold.initialize(
				&manifold,
				self.m_transform_a,
				self.m_polygon_a.base.m_radius,
				self.m_transform_b,
				self.m_polygon_b.base.m_radius,
			);

			let mut base = self.base.borrow_mut();

			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				&format!("point count = {0}", manifold.point_count),
			);
			base.m_text_line += base.m_text_increment;

			{
				let color = B2color::new(0.9, 0.9, 0.9);
				let mut v = <[B2vec2; B2_MAX_POLYGON_VERTICES]>::default();
				for i in 0..self.m_polygon_a.m_count {
					v[i] = b2_mul_transform_by_vec2(
						self.m_transform_a,
						self.m_polygon_a.m_vertices[i],
					);
				}
				base.g_debug_draw
					.borrow_mut()
					.draw_polygon(&v[..self.m_polygon_a.m_count], color);

				for i in 0..self.m_polygon_b.m_count {
					v[i] = b2_mul_transform_by_vec2(
						self.m_transform_b,
						self.m_polygon_b.m_vertices[i],
					);
				}
				base.g_debug_draw
					.borrow_mut()
					.draw_polygon(&v[..self.m_polygon_b.m_count], color);
			}

			for i in 0..manifold.point_count {
				base.g_debug_draw.borrow_mut().draw_point(
					world_manifold.points[i],
					4.0,
					B2color::new(0.9, 0.3, 0.3),
				);
			}
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}

//static int testIndex = RegisterTest("Geometry", "Polygon Collision", PolygonCollision::create);
