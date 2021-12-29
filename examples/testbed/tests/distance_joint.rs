use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_joint::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::joints::b2_distance_joint::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

use imgui::im_str;
use imgui::sys;

use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};

// This tests distance joints, body destruction, and joint destruction.
pub(crate) struct DistanceJoint<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,
	test_data: Rc<RefCell<TestData<D>>>,
}

#[derive(Default)]
struct TestData<D: UserDataType> {
	m_joint: Option<B2jointPtr<D>>,
	m_length: f32,
	m_minLength: f32,
	m_maxLength: f32,
	m_hertz: f32,
	m_dampingRatio: f32,

	m_world: Option<B2worldPtr<D>>,
}

impl<D: UserDataType> DistanceJoint<D> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<D, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let world = base.borrow().m_world.clone();

		let test_data = Rc::new(RefCell::new(TestData::default()));
		test_data.borrow_mut().m_world = Some(world.clone());

		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: Rc::new(RefCell::new(B2testContactListenerDefault {
				base: Rc::downgrade(&base),
			})),
			test_data: test_data.clone(),
		}));

		{
			let mut self_ = result_ptr.borrow_mut();
			{
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
			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;
			bd.angular_damping = 0.1;

			bd.position.set(0.0, 5.0);
			let body = B2world::create_body(m_world.clone(), &bd);

			let mut shape = B2polygonShape::default();
			shape.set_as_box(0.5, 0.5);
			B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 5.0);

			let mut test_data = self.test_data.borrow_mut();
			test_data.m_hertz = 1.0;
			test_data.m_dampingRatio = 0.7;

			let mut jd = B2distanceJointDef::default();
			jd.initialize(ground, body, B2vec2::new(0.0, 15.0), bd.position);
			jd.base.collide_connected = true;
			test_data.m_length = jd.length;
			test_data.m_minLength = test_data.m_length;
			test_data.m_maxLength = test_data.m_length;

			b2_linear_stiffness(
				&mut jd.stiffness,
				&mut jd.damping,
				test_data.m_hertz,
				test_data.m_dampingRatio,
				jd.base.body_a.clone().unwrap(),
				jd.base.body_b.clone().unwrap(),
			);

			test_data.m_joint = Some(
				m_world
					.borrow_mut()
					.create_joint(&B2JointDefEnum::DistanceJoint(jd.clone())),
			);
		}
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for DistanceJoint<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn update_ui(&mut self, ui: &imgui::Ui<'_>) {
		imgui::Window::new(im_str!("Joint Controls"))
			.flags(imgui::WindowFlags::NO_MOVE | imgui::WindowFlags::NO_RESIZE)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([260.0, 150.0], imgui::Condition::Always)
			.build(&ui, || unsafe {
				let mut test_data = self.test_data.borrow_mut();

				match test_data
					.m_joint
					.clone()
					.unwrap()
					.borrow_mut()
					.as_derived_mut()
				{
					JointAsDerivedMut::EDistanceJoint(ref mut m_joint) => {
						if sys::igSliderFloat(
							im_str!("Length").as_ptr(),
							&mut test_data.m_length,
							0.0,
							20.0,
							im_str!("%.0f").as_ptr(),
							1.0,
						) {
							test_data.m_length = m_joint.set_length(test_data.m_length);
						}

						if sys::igSliderFloat(
							im_str!("Min Length").as_ptr(),
							&mut test_data.m_minLength,
							0.0,
							20.0,
							im_str!("%.0f").as_ptr(),
							1.0,
						) {
							test_data.m_minLength = m_joint.SetMinLength(test_data.m_minLength);
						}

						if sys::igSliderFloat(
							im_str!("Max Length").as_ptr(),
							&mut test_data.m_maxLength,
							0.0,
							20.0,
							im_str!("%.0f").as_ptr(),
							1.0,
						) {
							test_data.m_maxLength = m_joint.SetMaxLength(test_data.m_maxLength);
						}

						if sys::igSliderFloat(
							im_str!("Hertz").as_ptr(),
							&mut test_data.m_hertz,
							0.0,
							10.0,
							im_str!("%.1f").as_ptr(),
							1.0,
						) {
							let mut stiffness = 0.0;
							let mut damping = 0.0;
							b2_linear_stiffness(
								&mut stiffness,
								&mut damping,
								test_data.m_hertz,
								test_data.m_dampingRatio,
								m_joint.get_base().get_body_a(),
								m_joint.get_base().get_body_b(),
							);
							m_joint.set_stiffness(stiffness);
							m_joint.set_damping(damping);
						}

						if sys::igSliderFloat(
							im_str!("Damping Ratio").as_ptr(),
							&mut test_data.m_dampingRatio,
							0.0,
							2.0,
							im_str!("%.1f").as_ptr(),
							1.0,
						) {
							let mut stiffness = 0.0;
							let mut damping = 0.0;
							b2_linear_stiffness(
								&mut stiffness,
								&mut damping,
								test_data.m_hertz,
								test_data.m_dampingRatio,
								m_joint.get_base().get_body_a(),
								m_joint.get_base().get_body_b(),
							);
							m_joint.set_stiffness(stiffness);
							m_joint.set_damping(damping);
						}
					}
					_ => (),
				}
			});
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
			"This demonstrates a soft distance joint.",
		);
		base.m_text_line += base.m_text_increment;
		base.g_debug_draw.borrow().draw_string(
			ui,
			B2vec2::new(5.0, base.m_text_line as f32),
			"Press: (b) to delete a body, (j) to delete a joint",
		);
		base.m_text_line += base.m_text_increment;
	}
}
