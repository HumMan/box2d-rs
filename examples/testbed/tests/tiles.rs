use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_timer::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
pub(crate) struct Tiles<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_fixture_count: i32,
	m_create_time: f32,
}

impl<D: UserDataType> Tiles<D> {
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
			m_fixture_count: 0,
			m_create_time: 0.0,
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

	const E_COUNT: usize = 20;

	fn init(&mut self) {
		let m_world = self.base.borrow().m_world.clone();

		self.m_fixture_count = 0;
		let timer = B2timer::default();

		{
			let a: f32 = 0.5;
			let mut bd = B2bodyDef::default();
			bd.position.y = -a;
			let ground = B2world::create_body(m_world.clone(), &bd);

			if true {
				const N: i32 = 200;
				const M: i32 = 10;
				let mut position = B2vec2::default();
				position.y = 0.0;
				for _j in 0..M {
					position.x = -N as f32 * a;
					for _i in 0..N {
						let mut shape = B2polygonShape::default();
						shape.set_as_box_angle(a, a, position, 0.0);
						B2body::create_fixture_by_shape(
							ground.clone(),
							Rc::new(RefCell::new(shape)),
							0.0,
						);
						self.m_fixture_count += 1;
						position.x += 2.0 * a;
					}
					position.y -= 2.0 * a;
				}
			} else {
				const N: i32 = 200;
				const M: i32 = 10;
				let mut position = B2vec2::default();
				position.x = -N as f32 * a;
				for _i in 0..N {
					position.y = 0.0;
					for _j in 0..M {
						let mut shape = B2polygonShape::default();
						shape.set_as_box_angle(a, a, position, 0.0);
						B2body::create_fixture_by_shape(
							ground.clone(),
							Rc::new(RefCell::new(shape)),
							0.0,
						);
						position.y -= 2.0 * a;
					}
					position.x += 2.0 * a;
				}
			}
		}

		{
			let a: f32 = 0.5;
			let mut shape = B2polygonShape::default();
			shape.set_as_box(a, a);

			let mut x = B2vec2::new(-7.0, 0.75);
			let mut y;
			let delta_x = B2vec2::new(0.5625, 1.25);
			let delta_y = B2vec2::new(1.125, 0.0);

			for i in 0..Self::E_COUNT {
				y = x;

				for _j in i..Self::E_COUNT {
					let mut bd = B2bodyDef::default();
					bd.body_type = B2bodyType::B2DynamicBody;
					bd.position = y;

					//if i == 0 && j == 0
					//{
					//	bd.allow_sleep = false;
					//}
					//else
					//{
					//	bd.allow_sleep = true;
					//}

					let body = B2world::create_body(m_world.clone(), &bd);
					B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 5.0);
					self.m_fixture_count += 1;
					y += delta_y;
				}

				x += delta_x;
			}
		}

		self.m_create_time = timer.get_milliseconds();
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Tiles<D> {
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
		{
			let m_world = self.base.borrow().m_world.clone();
			let mut base = self.base.borrow_mut();
			let cm = m_world.borrow().get_contact_manager();
			let height: i32 = cm.borrow().get_broad_phase().borrow().get_tree_height();
			let leaf_count: i32 = cm.borrow().get_broad_phase().borrow().get_proxy_count();
			let minimum_node_count: i32 = 2 * leaf_count - 1;
			let minimum_height: f32 = f32::ceil(f32::ln(minimum_node_count as f32) / f32::ln(2.0));

			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				&format!(
					"dynamic tree height = {0}, min = {1}",
					height, minimum_height as f32
				),
			);
			base.m_text_line += base.m_text_increment;
		}
		Test::step(self.base.clone(), ui, display, target, settings, *camera);
		{
			let mut base = self.base.borrow_mut();
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				&format!(
					"create time = {0:6.2} ms, fixture count = {1}",
					self.m_create_time, self.m_fixture_count
				),
			);
			base.m_text_line += base.m_text_increment;
		}

		//B2dynamicTree* tree = &m_world->m_contactManager.m_broadPhase.m_tree;

		//if m_step_count == 400
		//{
		//	tree->RebuildBottomUp();
		//}
	}
}

//static int testIndex = RegisterTest("Benchmark", "Tiles", Tiles::create);
