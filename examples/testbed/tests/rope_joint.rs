use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::joints::b2_revolute_joint::*;
use box2d_rs::joints::b2_rope_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

/// This test shows how a rope joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the rope joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the rope joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
pub(crate) struct RopeJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_rope_def: B2ropeJointDef<D>,
	m_rope: Option<B2jointPtr<D>>,
}

impl<D: UserDataType> RopeJoint<D> {
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

			m_rope_def: B2ropeJointDef::default(),
			m_rope: None,
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

		let ground: BodyPtr<D>;
		{
			let bd = B2bodyDef::default();
			ground = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2edgeShape::default();
			shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
			B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
		}

		{
			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 0.125);

			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 20.0;
			fd.friction = 0.2;
			fd.filter.category_bits = 0x0001;
			fd.filter.mask_bits = 0xFFFF & !0x0002;

			let mut jd = B2revoluteJointDef::default();
			jd.base.collide_connected = false;

			let n: i32 = 10;
			let y: f32 = 15.0;
			self.m_rope_def.local_anchor_a.set(0.0, y);

			let mut prev_body: BodyPtr<D> = ground.clone();
			for i in 0..n {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(0.5 + 1.0 * i as f32, y);
				if i == n - 1 {
					shape.set_as_box(1.5, 1.5);
					fd.density = 100.0;
					fd.filter.category_bits = 0x0002;
					bd.position.set(1.0 * i as f32, y);
					bd.angular_damping = 0.4;
				}

				let body = B2world::create_body(m_world.clone(), &bd);

				B2body::create_fixture(body.clone(), &fd);

				let anchor = B2vec2::new(i as f32, y);
				jd.initialize(prev_body, body.clone(), anchor);
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd.clone()));

				prev_body = body;
			}

			self.m_rope_def.local_anchor_b.set_zero();

			let extra_length: f32 = 0.01;
			self.m_rope_def.max_length = n as f32 - 1.0 + extra_length;
			self.m_rope_def.base.body_b = Some(prev_body);
		}

		{
			self.m_rope_def.base.body_a = Some(ground);
			self.m_rope = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RopeJoint(self.m_rope_def.clone())),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for RopeJoint<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::J) => {
					let m_world = self.base.borrow().m_world.clone();
					if let Some(m_rope) = self.m_rope.take() {
						m_world.borrow_mut().destroy_joint(m_rope);
					} else {
						self.m_rope = Some(
							m_world
								.borrow_mut()
								.create_joint(&B2JointDefEnum::RopeJoint(self.m_rope_def.clone())),
						);
					}
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

		let mut base = self.base.borrow_mut();

		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press (j) to toggle the rope joint.",
		);
		base.m_text_line += base.m_text_increment;

		if self.m_rope.is_some() {
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Rope ON",
			);
		} else {
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Rope OFF",
			);
		}
		base.m_text_line += base.m_text_increment;
	}
}

//static int testIndex = RegisterTest("Joints", "Rope", RopeJoint::create);
