/// This tests stacking. It also shows how to use b2World::query
/// and b2_test_overlap.
use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_shape::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

// This is a fun demo that shows off the wheel joint
pub(crate) struct PolygonShapes<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_bodies: Vec<BodyPtr<D>>,
	m_polygons: [B2polygonShape; 4],
	m_circle: B2circleShape,
}

const E_MAX_COUNT: usize = 4;

/// This callback is called by b2World::QueryAABB. We find all the fixtures
/// that overlap an AABB. Of those, we use b2_test_overlap to determine which fixtures
/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
fn report_fixture<D: UserDataType>(
	fixture: FixturePtr<D>,
	m_circle: Rc<B2circleShape>,
	m_transform: B2Transform,
	g_debug_draw: TestBedDebugDrawPtr,
	m_count: &mut usize,
) -> bool {
	if *m_count == E_MAX_COUNT {
		return false;
	}

	let body: BodyPtr<D> = fixture.borrow().get_body();
	let shape = fixture.borrow().get_shape();

	let overlap: bool = b2_test_overlap_shapes(
		shape,
		0,
		m_circle.clone(),
		0,
		body.borrow().get_transform(),
		m_transform,
	);

	if overlap {
		let color = B2color::new(0.95, 0.95, 0.6);
		let center: B2vec2 = body.borrow().get_world_center();
		g_debug_draw.borrow_mut().draw_point(center, 5.0, color);
		*m_count += 1;
	}

	return true;
}

impl<D: UserDataType> PolygonShapes<D> {
	pub const E_MAX_BODIES: usize = 256;

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
			m_bodies: Vec::with_capacity(Self::E_MAX_BODIES),
			m_polygons: Default::default(),
			m_circle: B2circleShape::default(),
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

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
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
	}

	fn create(&mut self, index: usize) {
		let m_world = self.base.borrow().m_world.clone();
		let mut bd = B2bodyDef::default();
		bd.body_type = B2bodyType::B2DynamicBody;

		let x: f32 = random_float_range(-2.0, 2.0);
		bd.position.set(x, 10.0);
		bd.angle = random_float_range(-B2_PI, B2_PI);

		if index == 4 {
			bd.angular_damping = 0.02;
		}

		let new_body = B2world::create_body(m_world.clone(), &bd);

		if index < 4 {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_polygons[index])));
			fd.density = 1.0;
			fd.friction = 0.3;
			B2body::create_fixture(new_body.clone(), &fd);
		} else {
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(self.m_circle)));
			fd.density = 1.0;
			fd.friction = 0.3;

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

impl<D: UserDataType, F: Facade> TestDyn<D, F> for PolygonShapes<D> {
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
				Some(VirtualKeyCode::A) => {
					for b in self.m_bodies.iter().step_by(2) {
						let enabled: bool = b.borrow().is_enabled();
						B2body::set_enabled(b.clone(), !enabled);
					}
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
		ui: &imgui::Ui<'_>,
		display: &F,
		target: &mut glium::Frame,
		settings: &mut Settings,
		camera: &mut Camera,
	) {
		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		let mut m_circle = B2circleShape::default();
		m_circle.base.m_radius = 2.0;
		m_circle.m_p.set(0.0, 1.1);
		let mut m_transform = B2Transform::default();
		m_transform.set_identity();

		let mut aabb = B2AABB::default();
		m_circle.compute_aabb(&mut aabb, m_transform, 0);

		let m_circle = Rc::new(m_circle);

		let m_world = self.base.borrow().m_world.clone();

		let mut m_count: usize = 0;

		let mut base = self.base.borrow_mut();

		m_world.borrow().query_aabb(
			|fixture: FixturePtr<D>| -> bool {
				return report_fixture(
					fixture,
					m_circle.clone(),
					m_transform,
					base.g_debug_draw.clone(),
					&mut m_count,
				);
			},
			aabb,
		);
		let color = B2color::new(0.4, 0.7, 0.8);
		base.g_debug_draw
			.borrow_mut()
			.draw_circle(m_circle.m_p, m_circle.base.m_radius, color);

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			&format!(
				"Press 1-5 to drop stuff, maximum of {0} overlaps detected",
				E_MAX_COUNT
			),
		);
		base.m_text_line += base.m_text_increment;

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press 'a' to enable/disable some bodies",
		);
		base.m_text_line += base.m_text_increment;

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press 'd' to destroy a body",
		);
		base.m_text_line += base.m_text_increment;
	}
}

//static int testIndex = RegisterTest("Geometry", "Polygon Shapes", PolygonShapes::create);
