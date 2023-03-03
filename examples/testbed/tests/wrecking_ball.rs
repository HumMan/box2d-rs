use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_polygon_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::joints::b2_distance_joint::*;
use box2d_rs::joints::b2_revolute_joint::*;




use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

/// This test shows how a distance joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the distance joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the distance joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
pub(crate) struct WreckingBall<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_distance_joint_def: B2distanceJointDef<D>,
	m_distance_joint: Option<B2jointPtr<D>>,
	m_stabilize: bool
}

impl<D: UserDataType> WreckingBall<D> {
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
			m_distance_joint_def: B2distanceJointDef::default(),
			m_distance_joint: None,
			m_stabilize: true,
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
	fn init(&mut self)
	{
		let ground: BodyPtr<D>;
		let m_world = self.base.borrow().m_world.clone();
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

			const N: i32 = 10;
			const Y: f32 = 15.0;
			self.m_distance_joint_def.local_anchor_a.set(0.0, Y);

			let mut prev_body = ground.clone();
			for i in 0..N
			{
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(0.5 + 1.0 * i as f32, Y);
				if i == N - 1
				{
					bd.position.set(1.0 * i as f32, Y);
					bd.angular_damping = 0.4;
				}

				let body = B2world::create_body(m_world.clone(), &bd);

				if i == N - 1
				{
					let mut circle_shape = B2circleShape::default();
					circle_shape.base.m_radius = 1.5;
					let mut sfd = B2fixtureDef::default();
					sfd.shape = Some(Rc::new(RefCell::new(circle_shape)));
					sfd.density = 100.0;
					sfd.filter.category_bits = 0x0002;
					B2body::create_fixture(body.clone(), &sfd);
				}
				else
				{
					B2body::create_fixture(body.clone(), &fd);
				}

				let anchor = B2vec2::new(i as f32, Y);
				jd.initialize(prev_body, body.clone(), anchor);
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::RevoluteJoint(jd.clone()));

				prev_body = body;
			}

			self.m_distance_joint_def.local_anchor_b.set_zero();

			let extra_length: f32 = 0.01;
			self.m_distance_joint_def.min_length = 0.0;
			self.m_distance_joint_def.max_length = (N as f32) - 1.0 + extra_length;
			self.m_distance_joint_def.base.body_b = Some(prev_body);
		}

		{
			self.m_distance_joint_def.base.body_a = Some(ground);
			self.m_distance_joint = Some(m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(self.m_distance_joint_def.clone())));
			self.m_stabilize = true;
		}
	}
}


impl<D: UserDataType, F: Facade> TestDyn<D, F> for WreckingBall<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn update_ui(&mut self, ui: &imgui::Ui) {
		ui.window("Wrecking Ball Controls")
			.flags(
				imgui::WindowFlags::NO_MOVE
					| imgui::WindowFlags::NO_RESIZE
			)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 100.0], imgui::Condition::Always)
			.build(|| {
				if ui.checkbox("Stabilize", &mut self.m_stabilize) {
					let m_world = self.base.borrow().m_world.clone();
					if self.m_stabilize == true && self.m_distance_joint.is_none()
					{
						self.m_distance_joint = Some(m_world.borrow_mut().create_joint(&B2JointDefEnum::DistanceJoint(self.m_distance_joint_def.clone())));
					}
					else if self.m_stabilize == false && self.m_distance_joint.is_some()
					{
						match self.m_distance_joint.take()
						{
							Some(j)=>{
								m_world.borrow_mut().destroy_joint(j);
							}
							None=>{}
						}
						
					}
				}
			});
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
		let mut base = self.base.borrow_mut();
		if self.m_distance_joint.is_some()
		{
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Distance Joint ON",
			);
		}
		else
		{
			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Distance Joint ON",
			);
			
		}
		base.m_text_line += base.m_text_increment;
	}
}

//static int testIndex = RegisterTest("Examples", "Wrecking Ball", WreckingBall::Create);
