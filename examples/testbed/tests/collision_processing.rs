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
use box2d_rs::shapes::b2_polygon_shape::*;

use itertools::Itertools;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// This test shows collision processing and tests
// deferred body destruction.
pub(crate) struct CollisionProcessing<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> CollisionProcessing<D> {
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

		// Ground body
		{
			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-50.0, 0.0), B2vec2::new(50.0, 0.0));

			let mut sd = B2fixtureDef::default();
			sd.shape = Some(Rc::new(RefCell::new(shape)));

			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);
			B2body::create_fixture(ground.clone(), &sd);
		}

		let x_lo: f32 = -5.0;
		let x_hi: f32 = 5.0;
		let y_lo: f32 = 2.0;
		let y_hi: f32 = 35.0;

		// Small triangle
		let mut vertices: [B2vec2; 3] = [
			B2vec2::new(-1.0, 0.0),
			B2vec2::new(1.0, 0.0),
			B2vec2::new(0.0, 2.0),
		];

		let polygon = Rc::new(RefCell::new(B2polygonShape::default()));
		polygon.borrow_mut().set(&vertices);

		let mut triangle_shape_def = B2fixtureDef::default();
		triangle_shape_def.shape = Some(polygon.clone());
		triangle_shape_def.density = 1.0;

		let mut triangle_body_def = B2bodyDef::default();
		triangle_body_def.body_type = B2bodyType::B2DynamicBody;
		triangle_body_def.position.set(
			random_float_range(x_lo, x_hi),
			random_float_range(y_lo, y_hi),
		);

		let body1 = B2world::create_body(m_world.clone(), &triangle_body_def);
		B2body::create_fixture(body1.clone(), &triangle_shape_def);

		// Large triangle (recycle definitions)
		vertices[0] *= 2.0;
		vertices[1] *= 2.0;
		vertices[2] *= 2.0;
		polygon.borrow_mut().set(&vertices);

		triangle_body_def.position.set(
			random_float_range(x_lo, x_hi),
			random_float_range(y_lo, y_hi),
		);

		let body2 = B2world::create_body(m_world.clone(), &triangle_body_def);
		B2body::create_fixture(body2.clone(), &triangle_shape_def);
		// Small box
		polygon.borrow_mut().set_as_box(1.0, 0.5);

		let mut box_shape_def = B2fixtureDef::default();
		box_shape_def.shape = Some(polygon.clone());
		box_shape_def.density = 1.0;

		let mut box_body_def = B2bodyDef::default();
		box_body_def.body_type = B2bodyType::B2DynamicBody;
		box_body_def.position.set(
			random_float_range(x_lo, x_hi),
			random_float_range(y_lo, y_hi),
		);

		let body3 = B2world::create_body(m_world.clone(), &box_body_def);
		B2body::create_fixture(body3.clone(), &box_shape_def);

		// Large box (recycle definitions)
		polygon.borrow_mut().set_as_box(2.0, 1.0);
		box_body_def.position.set(
			random_float_range(x_lo, x_hi),
			random_float_range(y_lo, y_hi),
		);
		let body4 = B2world::create_body(m_world.clone(), &box_body_def);
		B2body::create_fixture(body4.clone(), &box_shape_def);

		// Small circle
		let circle = Rc::new(RefCell::new(B2circleShape::default()));
		circle.borrow_mut().base.m_radius = 1.0;

		let mut circle_shape_def = B2fixtureDef::default();
		circle_shape_def.shape = Some(circle.clone());
		circle_shape_def.density = 1.0;

		let mut circle_body_def = B2bodyDef::default();
		circle_body_def.body_type = B2bodyType::B2DynamicBody;
		circle_body_def.position.set(
			random_float_range(x_lo, x_hi),
			random_float_range(y_lo, y_hi),
		);

		let body5 = B2world::create_body(m_world.clone(), &circle_body_def);
		B2body::create_fixture(body5.clone(), &circle_shape_def);

		// Large circle
		circle.borrow_mut().base.m_radius *= 2.0;
		circle_body_def.position.set(
			random_float_range(x_lo, x_hi),
			random_float_range(y_lo, y_hi),
		);

		let body6 = B2world::create_body(m_world.clone(), &circle_body_def);
		B2body::create_fixture(body6.clone(), &circle_shape_def);
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for CollisionProcessing<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
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

		// We are going to destroy some bodies according to contact
		// points. We must buffer the bodies that should be destroyed
		// because they may belong to multiple contact points.
		const K_MAX_NUKE: usize = 6;
		let mut nuke: Vec<BodyPtr<D>> = Vec::with_capacity(K_MAX_NUKE);

		// Traverse the contact results. destroy bodies that
		// are touching heavier bodies.
		for point in self.base.borrow().m_points.iter() {
			let body1: BodyPtr<D> = point.fixture_a.borrow().get_body();
			let body2: BodyPtr<D> = point.fixture_b.borrow().get_body();
			let mass1: f32 = body1.borrow().get_mass();
			let mass2: f32 = body2.borrow().get_mass();

			if mass1 > 0.0 && mass2 > 0.0 {
				if mass2 > mass1 {
					nuke.push(body1);
				} else {
					nuke.push(body2);
				}

				if nuke.len() == K_MAX_NUKE {
					break;
				}
			}
		}

		// Sort the nuke array to group duplicates.
		nuke = nuke.into_iter().unique_by(|a| Rc::as_ptr(a)).collect();

		// destroy the bodies, skipping duplicates.
		let m_bomb = self.base.borrow().m_bomb.clone();
		let m_world = self.base.borrow().m_world.clone();
		for b in nuke {
			if m_bomb.is_none() || !Rc::ptr_eq(&b, &m_bomb.as_ref().unwrap()) {
				m_world.borrow_mut().destroy_body(b);
			}
		}
	}
}
