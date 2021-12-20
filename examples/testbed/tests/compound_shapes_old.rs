use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// TODO_ERIN test joints on compounds.
pub(crate) struct CompoundShapesOld<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> CompoundShapesOld<D> {
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
			let mut bd = B2bodyDef::default();
			bd.position.set(0.0, 0.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(50.0, 0.0), B2vec2::new(-50.0, 0.0));

			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut circle1 = B2circleShape::default();
			circle1.base.m_radius = 0.5;
			circle1.m_p.set(-0.5, 0.5);

			let mut circle2 = B2circleShape::default();
			circle2.base.m_radius = 0.5;
			circle2.m_p.set(0.5, 0.5);

			for i in 0..10 {
				let x: f32 = random_float_range(-0.1, 0.1);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(x + 5.0, 1.05 + 2.5 * i as f32);
				bd.angle = random_float_range(-B2_PI, B2_PI);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(circle1)), 2.0);
				B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(circle2)), 0.0);
			}
		}

		{
			let mut polygon1 = B2polygonShape::default();
			polygon1.set_as_box(0.25, 0.5);

			let mut polygon2 = B2polygonShape::default();
			polygon2.set_as_box_angle(0.25, 0.5, B2vec2::new(0.0, -0.5), 0.5 * B2_PI);

			for i in 0..10 {
				let x: f32 = random_float_range(-0.1, 0.1);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(x - 5.0, 1.05 + 2.5 * i as f32);
				bd.angle = random_float_range(-B2_PI, B2_PI);
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(polygon1)), 2.0);
				B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(polygon2)), 2.0);
			}
		}

		{
			let mut xf1 = B2Transform::default();
			xf1.q.set(0.3524 * B2_PI);
			xf1.p = xf1.q.get_xaxis();

			let mut vertices = [B2vec2::zero(); 3];

			let mut triangle1 = B2polygonShape::default();
			vertices[0] = b2_mul_transform_by_vec2(xf1, B2vec2::new(-1.0, 0.0));
			vertices[1] = b2_mul_transform_by_vec2(xf1, B2vec2::new(1.0, 0.0));
			vertices[2] = b2_mul_transform_by_vec2(xf1, B2vec2::new(0.0, 0.5));
			triangle1.set(&vertices);

			let mut xf2 = B2Transform::default();
			xf2.q.set(-0.3524 * B2_PI);
			xf2.p = -xf2.q.get_xaxis();

			let mut triangle2 = B2polygonShape::default();
			vertices[0] = b2_mul_transform_by_vec2(xf2, B2vec2::new(-1.0, 0.0));
			vertices[1] = b2_mul_transform_by_vec2(xf2, B2vec2::new(1.0, 0.0));
			vertices[2] = b2_mul_transform_by_vec2(xf2, B2vec2::new(0.0, 0.5));
			triangle2.set(&vertices);

			for i in 0..10 {
				let x: f32 = random_float_range(-0.1, 0.1);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(x, 2.05 + 2.5 * i as f32);
				bd.angle = 0.0;
				let body = B2world::create_body(m_world.clone(), &bd);
				B2body::create_fixture_by_shape(
					body.clone(),
					Rc::new(RefCell::new(triangle1)),
					2.0,
				);
				B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(triangle2)), 2.0);
			}
		}

		{
			let mut bottom = B2polygonShape::default();
			bottom.set_as_box(1.5, 0.15);

			let mut left = B2polygonShape::default();
			left.set_as_box_angle(0.15, 2.7, B2vec2::new(-1.45, 2.35), 0.2);

			let mut right = B2polygonShape::default();
			right.set_as_box_angle(0.15, 2.7, B2vec2::new(1.45, 2.35), -0.2);

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 2.0);
			let body = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(bottom)), 4.0);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(left)), 4.0);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(right)), 4.0);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for CompoundShapesOld<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
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
	}
}
