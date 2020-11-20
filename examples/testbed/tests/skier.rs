/*
Test case for collision/jerking issue.
*/

use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_chain_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Skier<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_skier: Option<BodyPtr<D>>,
	m_platform_width: f32,
	m_fixed_camera: bool,
	g_camera: Camera,
}

impl<D: UserDataType> Skier<D> {
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
			m_skier: None,
			m_platform_width: 0.0,
			m_fixed_camera: false,
			g_camera: Camera::default(),
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

			const PLATFORM_WIDTH: f32 = 8.0;

			/*
			First angle is from the horizontal and should be negative for a downward slope.
			Second angle is relative to the preceding slope, and should be positive, creating a kind of
			loose 'Z'-shape from the 3 edges.
			If A1 = -10, then A2 <= ~1.5 will result in the collision glitch.
			If A1 = -30, then A2 <= ~10.0 will result in the glitch.
			*/
			const ANGLE1_DEGREES: f32 = -30.0;
			const ANGLE2_DEGREES: f32 = 10.0;
			/*
			The larger the value of SLOPE_LENGTH, the less likely the glitch will show up.
			*/
			const SLOPE_LENGTH: f32 = 2.0;

			const SURFACE_FRICTION: f32 = 0.2;

			// Convert to radians
			const SLOPE1_INCLINE: f32 = -ANGLE1_DEGREES * B2_PI / 180.0;
			const SLOPE2_INCLINE: f32 = SLOPE1_INCLINE - ANGLE2_DEGREES * B2_PI / 180.0;
			//

			self.m_platform_width = PLATFORM_WIDTH;

			// Horizontal platform
			let v1 = B2vec2::new(-PLATFORM_WIDTH, 0.0);
			let v2 = B2vec2::new(0.0, 0.0);
			let v3 = B2vec2::new(
				SLOPE_LENGTH * f32::cos(SLOPE1_INCLINE),
				-SLOPE_LENGTH * f32::sin(SLOPE1_INCLINE),
			);
			let v4 = B2vec2::new(
				v3.x + SLOPE_LENGTH * f32::cos(SLOPE2_INCLINE),
				v3.y - SLOPE_LENGTH * f32::sin(SLOPE2_INCLINE),
			);
			let v5 = B2vec2::new(v4.x, v4.y - 1.0);

			let vertices: [B2vec2; 5] = [v5, v4, v3, v2, v1];

			let mut shape = B2chainShape::default();
			shape.create_loop(&vertices);
			let mut fd = B2fixtureDef::default();
			fd.shape = Some(Rc::new(RefCell::new(shape)));
			fd.density = 0.0;
			fd.friction = SURFACE_FRICTION;

			B2body::create_fixture(ground.clone(), &fd);
		}

		{
			//const BODY_WIDTH: f32 = 1.0;
			const BODY_HEIGHT: f32 = 2.5;
			const SKI_LENGTH: f32 = 3.0;

			/*
			Larger values for this seem to alleviate the issue to some extent.
			*/
			const SKI_THICKNESS: f32 = 0.3;

			const SKI_FRICTION: f32 = 0.0;
			const SKI_RESTITUTION: f32 = 0.15;

			let mut bd = B2bodyDef::default();
			bd.body_type = B2bodyType::B2DynamicBody;

			const INITIAL_Y: f32 = BODY_HEIGHT / 2.0 + SKI_THICKNESS;
			bd.position.set(-self.m_platform_width / 2.0, INITIAL_Y);

			let skier = B2world::create_body(m_world.clone(), &bd);

			let mut ski = B2polygonShape::default();
			let verts: [B2vec2; 4] = [
				B2vec2::new(-SKI_LENGTH / 2.0 - SKI_THICKNESS, -BODY_HEIGHT / 2.0),
				B2vec2::new(-SKI_LENGTH / 2.0, -BODY_HEIGHT / 2.0 - SKI_THICKNESS),
				B2vec2::new(SKI_LENGTH / 2.0, -BODY_HEIGHT / 2.0 - SKI_THICKNESS),
				B2vec2::new(SKI_LENGTH / 2.0 + SKI_THICKNESS, -BODY_HEIGHT / 2.0),
			];
			ski.set(&verts);

			let mut fd = B2fixtureDef::default();
			fd.density = 1.0;

			fd.friction = SKI_FRICTION;
			fd.restitution = SKI_RESTITUTION;

			fd.shape = Some(Rc::new(RefCell::new(ski)));
			B2body::create_fixture(skier.clone(), &fd);

			skier
				.borrow_mut()
				.set_linear_velocity(B2vec2::new(0.5, 0.0));

			self.m_skier = Some(skier);
		}

		self.g_camera.m_center = B2vec2::new(self.m_platform_width / 2.0, 0.0);
		self.g_camera.m_zoom = 0.4;
		self.m_fixed_camera = true;
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Skier<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn keyboard(&mut self, key: &KeyboardInput) {
		if key.state == ElementState::Pressed {
			match key.virtual_keycode {
				Some(VirtualKeyCode::C) => {
					self.m_fixed_camera = !self.m_fixed_camera;
					if self.m_fixed_camera {
						self.g_camera.m_center = B2vec2::new(self.m_platform_width / 2.0, 0.0);
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
		{
			let mut base = self.base.borrow_mut();

			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Keys: c = Camera fixed/tracking",
			);
			base.m_text_line += base.m_text_increment;
		}

		self.g_camera.m_zoom = camera.m_zoom;
		self.g_camera.m_width = camera.m_width;
		self.g_camera.m_height = camera.m_height;

		if !self.m_fixed_camera {
			self.g_camera.m_center = self.m_skier.as_ref().unwrap().borrow().get_position();
			Test::step(
				self.base.clone(),
				ui,
				display,
				target,
				settings,
				self.g_camera,
			);
		} else {
			Test::step(self.base.clone(), ui, display, target, settings, *camera);
		}
	}
}

//static int testIndex = RegisterTest("Bugs", "Skier", Skier::Create);
