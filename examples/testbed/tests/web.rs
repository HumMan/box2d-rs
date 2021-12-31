use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::joints::b2_distance_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};

use std::cell::RefCell;
use std::rc::Rc;

// Test distance joints, body destruction, and joint destruction.
pub(crate) struct Web<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: Option<B2destructionListenerPtr<D>>,
	contact_listener: B2contactListenerPtr<D>,
	test_data: Rc<RefCell<TestData<D>>>,
}

#[derive(Default)]
struct TestData<D: UserDataType> {
	m_world: Option<B2worldPtr<D>>,

	m_bodies: Vec<BodyPtr<D>>,
	m_joints: Vec<B2jointPtr<D>>,
}

struct B2DestructionListenerCustom<D: UserDataType> 
{
	pub(crate) base: B2testDestructionListenerDefault<D>,
	test_data: Rc<RefCell<TestData<D>>>,
}
impl<D: UserDataType> B2destructionListener<D> for B2DestructionListenerCustom<D> {
	fn say_goodbye_fixture(&mut self, fixture: FixturePtr<D>) {
		self.base.say_goodbye_fixture(fixture);
	}
	fn say_goodbye_joint(&mut self, joint: B2jointPtr<D>) {
		self.base.say_goodbye_joint(joint.clone());

		let mut test_data = self.test_data.borrow_mut();
		test_data.m_joints.retain(|j| !Rc::ptr_eq(&j, &joint));
	}
}

impl<D: UserDataType> Web<D> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let world = base.borrow().m_world.clone();
		let test_data = Rc::new(RefCell::new(TestData {
			m_world: None,
			m_bodies: Vec::with_capacity(8),
			m_joints: Vec::with_capacity(4),
		}));
		{
			let mut test_data = test_data.borrow_mut();
			test_data.m_world = Some(world.clone());
		}

		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: None,
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
			test_data: test_data.clone(),
		}));

		let destruction_listener = Rc::new(RefCell::new(B2DestructionListenerCustom {
            base: {
                B2testDestructionListenerDefault {
                    base: Rc::downgrade(&base),
                }
            },
            test_data: test_data.clone(),
        }));

        result_ptr.borrow_mut().destruction_listener = Some(destruction_listener.clone());

		{
			let mut self_ = result_ptr.borrow_mut();
			{
				let mut world = world.borrow_mut();
				world.set_destruction_listener(destruction_listener.clone());
				world.set_contact_listener(self_.contact_listener.clone());
				world.set_debug_draw(global_draw);
			}
			self_.init();
		}

		return result_ptr;
	}

	fn init(&mut self) {
		{
			let m_world = self.base.borrow().m_world.clone();
			let mut test_data = self.test_data.borrow_mut();

			let ground;
			{
				let bd = B2bodyDef::default();
				ground = B2world::create_body(m_world.clone(), &bd);
				let mut shape = B2edgeShape::default();
				shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
				B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
			}
			{
				let mut shape = B2polygonShape::default();
				shape.set_as_box(0.5, 0.5);
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-5.0, 5.0);

				test_data
					.m_bodies
					.push(B2world::create_body(m_world.clone(), &bd));
				B2body::create_fixture_by_shape(
					test_data.m_bodies.last().unwrap().clone(),
					Rc::new(RefCell::new(shape)),
					5.0,
				);
				bd.position.set(5.0, 5.0);
				test_data
					.m_bodies
					.push(B2world::create_body(m_world.clone(), &bd));
				B2body::create_fixture_by_shape(
					test_data.m_bodies.last().unwrap().clone(),
					Rc::new(RefCell::new(shape)),
					5.0,
				);
				bd.position.set(5.0, 15.0);
				test_data
					.m_bodies
					.push(B2world::create_body(m_world.clone(), &bd));
				B2body::create_fixture_by_shape(
					test_data.m_bodies.last().unwrap().clone(),
					Rc::new(RefCell::new(shape)),
					5.0,
				);
				bd.position.set(-5.0, 15.0);
				test_data
					.m_bodies
					.push(B2world::create_body(m_world.clone(), &bd));
				B2body::create_fixture_by_shape(
					test_data.m_bodies.last().unwrap().clone(),
					Rc::new(RefCell::new(shape)),
					5.0,
				);
				let mut jd = B2distanceJointDef::default();
				const FREQUENCY_HZ: f32 = 2.0;
				const DAMPING_RATIO: f32 = 0.0;

				let create_joint =
					|m_world: B2worldPtr<D>, jd: &mut B2distanceJointDef<D>| -> B2jointPtr<D> {
						let body_a = jd.base.body_a.as_ref().unwrap().clone();
						let body_b = jd.base.body_b.as_ref().unwrap().clone();
						let p1 = body_a.borrow().get_world_point(jd.local_anchor_a);
						let p2 = body_b.borrow().get_world_point(jd.local_anchor_b);
						let d = p2 - p1;
						jd.length = d.length();
						b2_linear_stiffness(
							&mut jd.stiffness,
							&mut jd.damping,
							FREQUENCY_HZ,
							DAMPING_RATIO,
							body_a,
							body_b,
						);
						return m_world
							.borrow_mut()
							.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone()));
					};
				jd.base.body_a = Some(ground.clone());
				jd.base.body_b = Some(test_data.m_bodies[0].clone());
				jd.local_anchor_a.set(-10.0, 0.0);
				jd.local_anchor_b.set(-0.5, -0.5);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(ground.clone());
				jd.base.body_b = Some(test_data.m_bodies[1].clone());
				jd.local_anchor_a.set(10.0, 0.0);
				jd.local_anchor_b.set(0.5, -0.5);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(ground.clone());
				jd.base.body_b = Some(test_data.m_bodies[2].clone());
				jd.local_anchor_a.set(10.0, 20.0);
				jd.local_anchor_b.set(0.5, 0.5);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(ground.clone());
				jd.base.body_b = Some(test_data.m_bodies[3].clone());
				jd.local_anchor_a.set(-10.0, 20.0);
				jd.local_anchor_b.set(-0.5, 0.5);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(test_data.m_bodies[0].clone());
				jd.base.body_b = Some(test_data.m_bodies[1].clone());
				jd.local_anchor_a.set(0.5, 0.0);
				jd.local_anchor_b.set(-0.5, 0.0);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(test_data.m_bodies[1].clone());
				jd.base.body_b = Some(test_data.m_bodies[2].clone());
				jd.local_anchor_a.set(0.0, 0.5);
				jd.local_anchor_b.set(0.0, -0.5);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(test_data.m_bodies[2].clone());
				jd.base.body_b = Some(test_data.m_bodies[3].clone());
				jd.local_anchor_a.set(-0.5, 0.0);
				jd.local_anchor_b.set(0.5, 0.0);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));

				jd.base.body_a = Some(test_data.m_bodies[3].clone());
				jd.base.body_b = Some(test_data.m_bodies[0].clone());
				jd.local_anchor_a.set(0.0, -0.5);
				jd.local_anchor_b.set(0.0, 0.5);
				test_data.m_joints.push(create_joint(m_world.clone(), &mut jd));
			}
		}
	}
}
impl<D: UserDataType, F: Facade> TestDyn<D, F> for Web<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::B) => {
					let mut body = None;
					let m_world;
					{
						let mut test_data = self.test_data.borrow_mut();
						m_world = test_data.m_world.as_ref().unwrap().clone();
						if test_data.m_bodies.len() > 0
						{
							body = Some(test_data.m_bodies[0].clone());
							test_data.m_bodies.drain(..1);
						}
					}
					match body {
						Some(b) => {
							m_world.borrow_mut().destroy_body(b);
						}
						None => {}
					}
				}
				Some(VirtualKeyCode::J) => {
					let mut joint = None;
					let m_world;
					{
						let mut test_data = self.test_data.borrow_mut();
						m_world = test_data.m_world.as_ref().unwrap().clone();
						if test_data.m_joints.len() > 0
						{
							joint = Some(test_data.m_joints[0].clone());
							test_data.m_joints.drain(..1);
						}
					}
					match joint {
						Some(j) => {
							m_world.borrow_mut().destroy_joint(j);
						}
						None => {}
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
			"Press: (b) to delete a body, (j) to delete a joint",
		);
		base.m_text_line += base.m_text_increment;
	}
}

//static int testIndex = RegisterTest("Examples", "Web", Web::Create);
