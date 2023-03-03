use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use box2d_rs::b2_distance::*;
use box2d_rs::b2_time_of_impact::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

use std::sync::atomic::Ordering;

pub(crate) struct BulletTest<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_body: Option<BodyPtr<D>>,
	m_bullet: Option<BodyPtr<D>>,
	m_x: f32,
}

impl<D: UserDataType> BulletTest<D> {
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
			m_body: None,
			m_bullet: None,
			m_x: 0.0,
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

			let mut edge = B2edgeShape::default();

			edge.set_two_sided(B2vec2::new(-10.0, 0.0), B2vec2::new(10.0, 0.0));
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(edge)), 0.0);

			let mut shape = B2polygonShape::default();
			shape.set_as_box_angle(0.2, 1.0, B2vec2::new(0.5, 1.0), 0.0);
			B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.position.set(0.0, 4.0);

			let mut box_shape = B2polygonShape::default();
			box_shape.set_as_box(2.0, 0.1);

			let m_body = B2world::create_body(m_world.clone(), &bd);
			self.m_body = Some(m_body.clone());
			B2body::create_fixture_by_shape(m_body, Rc::new(RefCell::new(box_shape)), 1.0);

			box_shape.set_as_box(0.25, 0.25);

			//m_x = random_float(-1.0, 1.0);
			self.m_x = 0.20352793;
			bd.position.set(self.m_x, 10.0);
			bd.bullet = true;

			let m_bullet = B2world::create_body(m_world.clone(), &bd);
			self.m_bullet = Some(m_bullet.clone());
			B2body::create_fixture_by_shape(
				m_bullet.clone(),
				Rc::new(RefCell::new(box_shape)),
				100.0,
			);

			m_bullet
				.borrow_mut()
				.set_linear_velocity(B2vec2::new(0.0, -50.0));
		}
	}
	fn launch(&mut self) {
		let mut m_body = self.m_body.as_ref().unwrap().borrow_mut();
		m_body.set_transform(B2vec2::new(0.0, 4.0), 0.0);
		m_body.set_linear_velocity(B2vec2::zero());
		m_body.set_angular_velocity(0.0);

		self.m_x = random_float_range(-1.0, 1.0);
		let mut m_bullet = self.m_bullet.as_ref().unwrap().borrow_mut();
		m_bullet.set_transform(B2vec2::new(self.m_x, 10.0), 0.0);
		m_bullet.set_linear_velocity(B2vec2::new(0.0, -50.0));
		m_bullet.set_angular_velocity(0.0);

		B2_GJK_CALLS.store(0, Ordering::SeqCst);
		B2_GJK_ITERS.store(0, Ordering::SeqCst);
		B2_GJK_MAX_ITERS.store(0, Ordering::SeqCst);

		B2_TOI_CALLS.store(0, Ordering::SeqCst);
		B2_TOI_ITERS.store(0, Ordering::SeqCst);
		B2_TOI_MAX_ITERS.store(0, Ordering::SeqCst);
		B2_TOI_ROOT_ITERS.store(0, Ordering::SeqCst);
		B2_TOI_MAX_ROOT_ITERS.store(0, Ordering::SeqCst);
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for BulletTest<D> {
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
			let mut base = self.base.borrow_mut();

			let load_b2_gjk_calls = B2_GJK_CALLS.load(Ordering::SeqCst);
			let load_b2_gjk_iters = B2_GJK_ITERS.load(Ordering::SeqCst);
			let load_b2_gjk_max_iters = B2_GJK_MAX_ITERS.load(Ordering::SeqCst);
			let load_b2_toi_calls = B2_TOI_CALLS.load(Ordering::SeqCst);
			let load_b2_toi_iters = B2_TOI_ITERS.load(Ordering::SeqCst);
			let load_b2_toi_max_root_iters = B2_TOI_MAX_ROOT_ITERS.load(Ordering::SeqCst);
			let load_b2_toi_root_iters = B2_TOI_ROOT_ITERS.load(Ordering::SeqCst);

			if load_b2_gjk_calls > 0 {
				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					&format!(
						"gjk calls = {0}, ave gjk iters = {1:3.1}, max gjk iters = {2}",
						load_b2_gjk_calls,
						load_b2_gjk_iters as f32 / load_b2_gjk_calls as f32,
						load_b2_gjk_max_iters
					),
				);
				base.m_text_line += base.m_text_increment;
			}

			if load_b2_toi_calls > 0 {
				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					&format!(
						"toi calls = {0}, ave toi iters = {1:3.1}, max toi iters = {2}",
						load_b2_toi_calls,
						load_b2_toi_iters as f32 / load_b2_toi_calls as f32,
						load_b2_toi_max_root_iters
					),
				);
				base.m_text_line += base.m_text_increment;

				base.g_debug_draw.borrow().draw_string(
					ui,
					B2vec2::new(5.0, base.m_text_line as f32),
					&format!(
						"ave toi root iters = {0:3.1}, max toi root iters = {1}",
						load_b2_toi_root_iters as f32 / load_b2_toi_calls as f32,
						load_b2_toi_max_root_iters
					),
				);
				base.m_text_line += base.m_text_increment;
			}
		}

		if self.base.borrow().m_step_count % 60 == 0 {
			self.launch();
		}

		Test::step(self.base.clone(), ui, display, target, settings, *camera);
	}
}
