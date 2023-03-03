use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_chain_shape::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on edge chains.
pub(crate) struct CharacterCollision<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_character: Option<BodyPtr<D>>,
}

impl<D: UserDataType> CharacterCollision<D> {
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

			m_character: None,
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
			shape.set_two_sided(B2vec2::new(-20.0, 0.0), B2vec2::new(20.0, 0.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		// Collinear edges with no adjacency information.
		// This shows the problematic case where a box shape can hit
		// an internal vertex.
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-8.0, 1.0), B2vec2::new(-6.0, 1.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
			shape.set_two_sided(B2vec2::new(-6.0, 1.0), B2vec2::new(-4.0, 1.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
			shape.set_two_sided(B2vec2::new(-4.0, 1.0), B2vec2::new(-2.0, 1.0));
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		// Chain shape
		{
			let mut bd = B2bodyDef::default();
			bd.angle = 0.25 * B2_PI;
			let ground = B2world::create_body(m_world.clone(), &bd);

			let vs: [B2vec2; 4] = [
				B2vec2::new(5.0, 7.0),
				B2vec2::new(6.0, 8.0),
				B2vec2::new(7.0, 8.0),
				B2vec2::new(8.0, 7.0),
			];
			let mut shape = B2chainShape::default();
			shape.create_loop(&vs);
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		// Square tiles. This shows that adjacency shapes may
		// have non-smooth collision. There is no solution
		// to this problem.
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box_angle(1.0, 1.0, B2vec2::new(4.0, 3.0), 0.0);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
			shape.set_as_box_angle(1.0, 1.0, B2vec2::new(6.0, 3.0), 0.0);
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
			shape.set_as_box_angle(1.0, 1.0, B2vec2::new(8.0, 3.0), 0.0);
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		// Square made from an edge loop. Collision should be smooth.
		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			let vs: [B2vec2; 4] = [
				B2vec2::new(-1.0, 3.0),
				B2vec2::new(1.0, 3.0),
				B2vec2::new(1.0, 5.0),
				B2vec2::new(-1.0, 5.0),
			];
			let mut shape = B2chainShape::default();
			shape.create_loop(&vs);
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		// Edge loop. Collision should be smooth.
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(-10.0, 4.0);
			let ground = B2world::create_body(m_world.clone(), &bd);

			let vs: [B2vec2; 10] = [
				B2vec2::new(0.0, 0.0),
				B2vec2::new(6.0, 0.0),
				B2vec2::new(6.0, 2.0),
				B2vec2::new(4.0, 1.0),
				B2vec2::new(2.0, 2.0),
				B2vec2::new(0.0, 2.0),
				B2vec2::new(-2.0, 2.0),
				B2vec2::new(-4.0, 3.0),
				B2vec2::new(-6.0, 2.0),
				B2vec2::new(-6.0, 0.0),
			];
			let mut shape = B2chainShape::default();
			shape.create_loop(&vs);
			B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
		}

		// Square character 1
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(-3.0, 8.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.fixed_rotation = true;
			bd.allow_sleep = false;

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 0.5);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			B2body::create_fixture(body.clone(), &fd);
		}

		// Square character 2
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(-5.0, 5.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.fixed_rotation = true;
			bd.allow_sleep = false;

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.25, 0.25);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			B2body::create_fixture(body.clone(), &fd);
		}

		// Hexagon character
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(-5.0, 8.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.fixed_rotation = true;
			bd.allow_sleep = false;

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut angle: f32 = 0.0;
			let delta: f32 = B2_PI / 3.0;
			let mut vertices = [B2vec2::zero(); 6];
			for i in 0..vertices.len() {
				vertices[i].set(0.5 * f32::cos(angle), 0.5 * f32::sin(angle));
				angle += delta;
			}

			let mut shape = B2polygonShape::default();
			shape.set(&vertices);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			B2body::create_fixture(body.clone(), &fd);
		}

		// Circle character
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(3.0, 5.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.fixed_rotation = true;
			bd.allow_sleep = false;

			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.5;

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			B2body::create_fixture(body.clone(), &fd);
		}

		// Circle character
		{
			let mut bd = B2bodyDef::default();
			bd.position.set(-7.0, 6.0);
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.allow_sleep = false;

			let m_character = B2world::create_body(m_world.clone(), &bd);
			self.m_character = Some(m_character.clone());

			let mut shape = B2circleShape::default();
			shape.base.m_radius = 0.25;

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			fd.friction = 1.0;
			B2body::create_fixture(m_character.clone(), &fd);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for CharacterCollision<D> {
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
		{
			let mut m_character = self.m_character.as_ref().unwrap().borrow_mut();
			let mut v: B2vec2 = m_character.get_linear_velocity();
			v.x = -5.0;
			m_character.set_linear_velocity(v);
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		{
			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"This tests various character collision shapes.",
			);
			base.m_text_line += base.m_text_increment;
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Limitation: square and hexagon can snag on aligned boxes.",
			);
			base.m_text_line += base.m_text_increment;
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Feature: edge chains have smooth collision inside and out.",
			);
			base.m_text_line += base.m_text_increment;
		}
	}
}
