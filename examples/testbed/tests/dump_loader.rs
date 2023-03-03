use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_shape::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_chain_shape::*;
use box2d_rs::shapes::b2_circle_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;
// This test holds worlds dumped using b2World::Dump.
pub(crate) struct DumpLoader<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_ball: Option<BodyPtr<D>>,
}

impl<D: UserDataType> DumpLoader<D> {
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
			m_ball: None,
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

		let mut chain_shape = B2chainShape::default();
		let vertices: [B2vec2; 6] = [
			B2vec2::new(-5.0, 0.0),
			B2vec2::new(5.0, 0.0),
			B2vec2::new(5.0, 5.0),
			B2vec2::new(4.0, 1.0),
			B2vec2::new(-4.0, 1.0),
			B2vec2::new(-5.0, 5.0),
		];
		chain_shape.create_loop(&vertices);

		let mut ground_fixture_def = B2fixtureDef::default();
		ground_fixture_def.density = 0.0;
		ground_fixture_def.shape = Some(Rc::new(RefCell::new(chain_shape)));

		let mut ground_body_def = B2bodyDef::default();
		ground_body_def.body_type = B2bodyType::B2StaticBody;

		let ground_body = B2world::create_body(m_world.clone(), &ground_body_def);
		let _ground_body_fixture = B2body::create_fixture(ground_body.clone(), &ground_fixture_def);

		let mut ball_shape = B2circleShape::default();
		ball_shape.base.m_radius = 1.0;

		let mut ball_fixture_def = B2fixtureDef::default();
		ball_fixture_def.restitution = 0.75;
		ball_fixture_def.density = 1.0;
		ball_fixture_def.shape = Some(Rc::new(RefCell::new(ball_shape)));

		let mut ball_body_def = B2bodyDef::default();
		ball_body_def.body_type = B2bodyType::B2DynamicBody;
		ball_body_def.position = B2vec2::new(0.0, 10.0);
		// ball_body_def.angular_damping = 0.2f;

		let m_ball = B2world::create_body(m_world.clone(), &ball_body_def);
		self.m_ball = Some(m_ball.clone());
		let _ball_fixture = B2body::create_fixture(m_ball.clone(), &ball_fixture_def);
		m_ball
			.borrow_mut()
			.apply_force_to_center(B2vec2::new(-1000.0, -400.0), true);
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for DumpLoader<D> {
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
			let m_ball = self.m_ball.clone().unwrap();
			let v: B2vec2 = m_ball.borrow().get_linear_velocity();
			let omega: f32 = m_ball.borrow().get_angular_velocity();

			let mut mass_data = B2massData::default();
			m_ball.borrow().get_mass_data(&mut mass_data);

			let ke: f32 = 0.5 * mass_data.mass * b2_dot(v, v) + 0.5 * mass_data.i * omega * omega;

			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				&format!("kinetic energy = {0:.6}", ke),
			);
			base.m_text_line += base.m_text_increment;
		}
		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}
