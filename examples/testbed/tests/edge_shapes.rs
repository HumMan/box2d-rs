use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;

use box2d_rs::b2_draw::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

const E_MAX_BODIES: usize = 256;

pub(crate) struct EdgeShapes<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_bodies: Vec<BodyPtr<D>>,
	m_polygons: [B2polygonShape; 4],
	m_circle: B2circleShape,

	m_angle: f32,
}

impl<D: UserDataType> EdgeShapes<D> {
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
			m_bodies: Vec::with_capacity(E_MAX_BODIES),
			m_polygons: [B2polygonShape::default(); 4],
			m_circle: B2circleShape::default(),
			m_angle: 0.0,
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
		// Ground body
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut x1: f32 = -20.0;
			let mut y1: f32 = 2.0 * f32::cos(x1 / 10.0 * B2_PI);
			for _i in 0..80 {
				let x2: f32 = x1 + 0.5;
				let y2: f32 = 2.0 * f32::cos(x2 / 10.0 * B2_PI);

				let mut shape = B2edgeShape::default();
				shape.set_two_sided(B2vec2::new(x1, y1), B2vec2::new(x2, y2));
				B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);

				x1 = x2;
				y1 = y2;
			}
		}

		{
			let vertices: [B2vec2; 3] = [
				B2vec2::new(-0.5, 0.0),
				B2vec2::new(0.5, 0.0),
				B2vec2::new(0.0, 1.5),
			];
			self.m_polygons[0].set(&vertices);
		}

		{
			let vertices: [B2vec2; 3] = [
				B2vec2::new(-0.1, 0.0),
				B2vec2::new(0.1, 0.0),
				B2vec2::new(0.0, 1.5),
			];
			self.m_polygons[1].set(&vertices);
		}

		{
			let w: f32 = 1.0;
			let b: f32 = w / (2.0 + b2_sqrt(2.0));
			let s: f32 = b2_sqrt(2.0) * b;

			let vertices: [B2vec2; 8] = [
				B2vec2::new(0.5 * s, 0.0),
				B2vec2::new(0.5 * w, b),
				B2vec2::new(0.5 * w, b + s),
				B2vec2::new(0.5 * s, w),
				B2vec2::new(-0.5 * s, w),
				B2vec2::new(-0.5 * w, b + s),
				B2vec2::new(-0.5 * w, b),
				B2vec2::new(-0.5 * s, 0.0),
			];

			self.m_polygons[2].set(&vertices);
		}

		{
			self.m_polygons[3].set_as_box(0.5, 0.5);
		}

		{
			self.m_circle.base.m_radius = 0.5;
		}

		self.m_angle = 0.0;
	}

	fn create(&mut self, index: usize) {
		let mut bd = B2bodyDef::default();

		let x: f32 = random_float_range(-10.0, 10.0);
		let y: f32 = random_float_range(10.0, 20.0);
		bd.position.set(x, y);
		bd.angle = random_float_range(-B2_PI, B2_PI);
		bd.body_type = B2bodyType::B2DynamicBody;

		if index == 4 {
			bd.angular_damping = 0.02;
		}

		let m_world = self.base.borrow().m_world.clone();
		let new_body = B2world::create_body(m_world, &bd);

		if index < 4 {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_polygons[index])));
			fd.friction = 0.3;
			fd.density = 20.0;
			B2body::create_fixture(new_body.clone(), &fd);
		} else {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_circle)));
			fd.friction = 0.3;
			fd.density = 20.0;
			B2body::create_fixture(new_body.clone(), &fd);
		}

		self.m_bodies.push(new_body);
	}

	fn destroy_body(&mut self) {
		if let Some(body) = self.m_bodies.pop() {
			self.base.borrow().m_world.borrow_mut().destroy_body(body);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for EdgeShapes<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::Key1) => {
					self.create(0);
				}
				Some(VirtualKeyCode::Key2) => {
					self.create(1);
				}
				Some(VirtualKeyCode::Key3) => {
					self.create(2);
				}
				Some(VirtualKeyCode::Key4) => {
					self.create(3);
				}
				Some(VirtualKeyCode::Key5) => {
					self.create(4);
				}
				Some(VirtualKeyCode::D) => {
					self.destroy_body();
				}
				_ => (),
			}
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
		let advance_ray: bool = settings.m_pause == false || settings.m_single_step;

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press 1-5 to drop stuff",
		);
		base.m_text_line += base.m_text_increment;

		let l: f32 = 25.0;
		let point1 = B2vec2::new(0.0, 10.0);
		let d = B2vec2::new(
			l * f32::cos(self.m_angle),
			-l * b2_abs(f32::sin(self.m_angle)),
		);
		let point2: B2vec2 = point1 + d;

		let m_world = base.m_world.clone();

		let mut ray_cast_collide = None;

		m_world.borrow().ray_cast(
			|_fixture: FixturePtr<D>, point: B2vec2, normal: B2vec2, fraction: f32| -> f32 {
				ray_cast_collide = Some((point, normal));
				return fraction;
			},
			point1,
			point2,
		);

		if let Some((point, normal)) = ray_cast_collide {
			base.g_debug_draw
				.borrow_mut()
				.draw_point(point, 5.0, B2color::new(0.4, 0.9, 0.4));

			base.g_debug_draw
				.borrow_mut()
				.draw_segment(point1, point, B2color::new(0.8, 0.8, 0.8));

			let head: B2vec2 = point + 0.5 * normal;
			base.g_debug_draw
				.borrow_mut()
				.draw_segment(point, head, B2color::new(0.9, 0.9, 0.4));
		} else {
			base.g_debug_draw.borrow_mut().draw_segment(
				point1,
				point2,
				B2color::new(0.8, 0.8, 0.8),
			);
		}

		if advance_ray {
			self.m_angle += 0.25 * B2_PI / 180.0;
		}
	}
}
