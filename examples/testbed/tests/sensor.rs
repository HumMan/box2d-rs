use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_contact::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

struct B2contactListenerCustom<D: UserDataType> {
	base: B2testContactListenerDefault<D>,
	test_data: Rc<RefCell<TestData<D>>>,
}

impl B2contactListener<UserDataTypes> for B2contactListenerCustom<UserDataTypes> {
	fn begin_contact(&mut self, contact: &mut dyn B2contactDynTrait<UserDataTypes>) {
		self.base.begin_contact(contact);

		let mut test_data = self.test_data.borrow_mut();
		let fixture_a = contact.get_base().get_fixture_a();
		let fixture_b = contact.get_base().get_fixture_b();

		if Rc::ptr_eq(&fixture_a, test_data.m_sensor.as_ref().unwrap()) {
			let body = fixture_b.borrow().get_body();
			let body = body.borrow_mut();
			if let Some(user_data) = body.get_user_data() {
				match user_data {
					FixtureData::Int(i) => {
						test_data.m_touching[i as usize] = true;
					}
				}
			}
		}

		if Rc::ptr_eq(&fixture_b, test_data.m_sensor.as_ref().unwrap()) {
			let body = fixture_a.borrow().get_body();
			let body = body.borrow_mut();
			if let Some(user_data) = body.get_user_data() {
				match user_data {
					FixtureData::Int(i) => {
						test_data.m_touching[i as usize] = true;
					}
				}
			}
		}
	}
	fn end_contact(&mut self, contact: &mut dyn B2contactDynTrait<UserDataTypes>) {

		self.base.end_contact(contact);

		let mut test_data = self.test_data.borrow_mut();
		let fixture_a = contact.get_base().get_fixture_a();
		let fixture_b = contact.get_base().get_fixture_b();

		if Rc::ptr_eq(&fixture_a, test_data.m_sensor.as_ref().unwrap()) {
			let body = fixture_b.borrow().get_body();
			let body = body.borrow_mut();
			if let Some(user_data) = body.get_user_data() {
				match user_data {
					FixtureData::Int(i) => {
						test_data.m_touching[i as usize] = false;
					}
				}
			}
		}

		if Rc::ptr_eq(&fixture_b, test_data.m_sensor.as_ref().unwrap()) {
			let body = fixture_a.borrow().get_body();
			let body = body.borrow_mut();
			if let Some(user_data) = body.get_user_data() {
				match user_data {
					FixtureData::Int(i) => {
						test_data.m_touching[i as usize] = false;
					}
				}
			}
		}
	}
	fn pre_solve(
		&mut self,
		contact: &mut dyn B2contactDynTrait<UserDataTypes>,
		old_manifold: &B2manifold,
	) {
		self.base.pre_solve(contact, old_manifold);
	}
	fn post_solve(
		&mut self,
		contact: &mut dyn B2contactDynTrait<UserDataTypes>,
		impulse: &B2contactImpulse,
	) {
		self.base.post_solve(contact, impulse);
	}
}

#[derive(Default)]
pub(crate) struct TestData<D: UserDataType> {
	pub(crate) m_world: Option<B2worldPtr<D>>,

	pub(crate) m_sensor: Option<FixturePtr<D>>,
	m_bodies: Vec<BodyPtr<D>>,
	m_touching: Vec<bool>,
	m_force: f32
}

pub(crate) struct Sensors<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: Option<B2contactListenerPtr<D>>,
	test_data: Rc<RefCell<TestData<D>>>,
}

const E_COUNT: usize = 7;

// This shows how to use sensor shapes. Sensors don't have collision, but report overlap events.
impl Sensors<UserDataTypes> {
	pub fn new<F: Facade>(global_draw: TestBedDebugDrawPtr) -> TestPtr<UserDataTypes, F> {
		let base = Rc::new(RefCell::new(Test::new(global_draw.clone())));
		let world = base.borrow().m_world.clone();

		let test_data = Rc::new(RefCell::new(TestData::default()));
		{
			let mut test_data = test_data.borrow_mut();
			test_data.m_world = Some(world.clone());
			test_data.m_bodies = Vec::with_capacity(E_COUNT);
			test_data.m_touching = Vec::with_capacity(E_COUNT);
		}

		let result_ptr = Rc::new(RefCell::new(Self {
			base: base.clone(),
			destruction_listener: Rc::new(RefCell::new(B2testDestructionListenerDefault {
				base: Rc::downgrade(&base),
			})),
			contact_listener: None,
			test_data: test_data.clone(),
		}));

		let contact_listener = Rc::new(RefCell::new(B2contactListenerCustom {
			base: {
				B2testContactListenerDefault {
					base: Rc::downgrade(&base),
				}
			},
			test_data: test_data.clone(),
		}));

		result_ptr.borrow_mut().contact_listener = Some(contact_listener.clone());

		{
			let mut self_ = result_ptr.borrow_mut();
			{
				let mut world = world.borrow_mut();
				world.set_destruction_listener(self_.destruction_listener.clone());
				world.set_contact_listener(contact_listener.clone());
				world.set_debug_draw(global_draw);
			}
			self_.init();
		}

		return result_ptr;
	}

	fn init(&mut self) {
		let m_world = self.base.borrow().m_world.clone();

		let mut test_data = self.test_data.borrow_mut();

		{
			let bd = B2bodyDef::default();
			let ground = B2world::create_body(m_world.clone(), &bd);

			{
				let mut shape = B2edgeShape::default();
				shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
				B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
			}

			if false {
				// let mut sd = B2fixtureDef::default();
				// sd.set_as_box_angle(10.0, 2.0, B2vec2::new(0.0, 20.0), 0.0);
				// sd.is_sensor = true;
				// m_sensor = B2body::create_fixture(ground.clone(), &sd);
			} else {
				let mut shape = B2circleShape::default();
				shape.base.m_radius = 5.0;
				shape.m_p.set(0.0, 10.0);

				let mut fd = B2fixtureDef::default();
				fd.shape = Some(Rc::new(RefCell::new(shape)));
				fd.is_sensor = true;
				test_data.m_sensor = Some(B2body::create_fixture(ground.clone(), &fd));
			}
		}

		{
			let mut shape = B2circleShape::default();
			shape.base.m_radius = 1.0;

			for i in 0..E_COUNT {
				let mut bd = B2bodyDef::default();
				bd.body_type = B2bodyType::B2DynamicBody;
				bd.position.set(-10.0 + 3.0 * i as f32, 20.0);
				bd.user_data = Some(FixtureData::Int(i as i32));

				test_data.m_touching.push(false);
				let body = B2world::create_body(m_world.clone(), &bd);

				B2body::create_fixture_by_shape(body.clone(), Rc::new(RefCell::new(shape)), 1.0);
				test_data.m_bodies.push(body);
			}
		}
		test_data.m_force = 100.0;
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Sensors<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn update_ui(&mut self, ui: &imgui::Ui) {
		ui.window("Sensor Controls")
			.flags(imgui::WindowFlags::NO_MOVE | imgui::WindowFlags::NO_RESIZE)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 60.0], imgui::Condition::Always)
			.build(|| {
				let mut test_data = self.test_data.borrow_mut();

				ui.slider_config("Force", 0.0, 2000.0)
						.display_format("%.0f")
						.build(&mut test_data.m_force);
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

		let test_data = self.test_data.borrow_mut();
		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for i in 0..E_COUNT {
			if test_data.m_touching[i] == false {
				continue;
			}

			let body: BodyPtr<D> = test_data.m_bodies[i].clone();
			let m_sensor = test_data.m_sensor.clone().unwrap();
			let ground: BodyPtr<D> = m_sensor.borrow().get_body();

			let circle_m_p;

			if let Some(circle) = m_sensor.borrow().get_shape().as_circle() {
				circle_m_p = circle.m_p;
			} else {
				panic!();
			}

			let center: B2vec2 = ground.borrow().get_world_point(circle_m_p);

			let position: B2vec2 = body.borrow().get_position();

			let mut d: B2vec2 = center - position;
			if d.length_squared() < B2_EPSILON * B2_EPSILON {
				continue;
			}

			d.normalize();
			let f: B2vec2 =test_data.m_force * d;
			body.borrow_mut().apply_force(f, position, false);
		}
	}
}

//static int testIndex = RegisterTest("Collision", "Sensors", Sensors::create);
