use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_rope::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world_callbacks::*;

use std::ffi::CString;

use imgui::Slider;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Rope<D: UserDataType> {
	base: TestBasePtr<D>,
	destruction_listener: B2destructionListenerPtr<D>,
	contact_listener: B2contactListenerPtr<D>,

	m_rope1: B2rope,
	m_rope2: B2rope,
	m_tuning1: B2ropeTuning,
	m_tuning2: B2ropeTuning,
	m_iterations1: i32,
	m_iterations2: i32,
	m_position1: B2vec2,
	m_position2: B2vec2,
	m_speed: f32,

	left_pressed: bool,
	right_pressed: bool,
}

impl<D: UserDataType> Rope<D> {
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

			m_rope1: Default::default(),
			m_rope2: Default::default(),
			m_tuning1: Default::default(),
			m_tuning2: Default::default(),
			m_iterations1: Default::default(),
			m_iterations2: Default::default(),
			m_position1: Default::default(),
			m_position2: Default::default(),
			m_speed: Default::default(),

			left_pressed: false,
			right_pressed: false,
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
		const N: usize = 20;
		let l: f32 = 0.5;
		let mut vertices = <[B2ropeDefVertices; N]>::default();

		for (i, v) in &mut vertices.iter_mut().enumerate() {
			v.position.set(0.0, l * (N - i) as f32);
			v.mass = 1.0;
		}
		vertices[0].mass = 0.0;
		vertices[1].mass = 0.0;

		self.m_tuning1.bend_hertz = 30.0;
		self.m_tuning1.bend_damping = 4.0;
		self.m_tuning1.bend_stiffness = 1.0;
		self.m_tuning1.bending_model = B2bendingModel::B2PbdTriangleBendingModel;
		self.m_tuning1.isometric = true;

		self.m_tuning1.stretch_hertz = 30.0;
		self.m_tuning1.stretch_damping = 4.0;
		self.m_tuning1.stretch_stiffness = 1.0;
		self.m_tuning1.stretching_model = B2stretchingModel::B2PbdStretchingModel;

		self.m_tuning2.bend_hertz = 30.0;
		self.m_tuning2.bend_damping = 0.7;
		self.m_tuning2.bend_stiffness = 1.0;
		self.m_tuning2.bending_model = B2bendingModel::B2PbdHeightBendingModel;
		self.m_tuning2.isometric = true;

		self.m_tuning2.stretch_hertz = 30.0;
		self.m_tuning2.stretch_damping = 1.0;
		self.m_tuning2.stretch_stiffness = 1.0;
		self.m_tuning2.stretching_model = B2stretchingModel::B2PbdStretchingModel;

		self.m_position1.set(-5.0, 15.0);
		self.m_position2.set(5.0, 15.0);

		let mut def = B2ropeDef::default();
		def.vertices = vertices.to_vec();
		def.gravity.set(0.0, -10.0);

		def.position = self.m_position1;
		def.tuning = self.m_tuning1;
		self.m_rope1.create(&def);

		def.position = self.m_position2;
		def.tuning = self.m_tuning2;
		self.m_rope2.create(&def);

		self.m_iterations1 = 8;
		self.m_iterations2 = 8;

		self.m_speed = 10.0;
	}
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Rope<D> {
	fn get_base(&self) -> TestBasePtr<D> {
		return self.base.clone();
	}
	fn update_ui(&mut self, ui: &imgui::Ui<'_>) {
		imgui::Window::new("Tuning")
			.flags(imgui::WindowFlags::NO_MOVE | imgui::WindowFlags::NO_RESIZE)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 700.0], imgui::Condition::Always)
			.build(&ui, || {
				ui.separator();
				//TODO_humman sys::igGetWindowWidth()
				let width_token = ui.push_item_width(200.0 * 0.5);

				let bend_models = ["Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height", "PBD Triangle"];
				let stretch_models = ["PBD", "XPBD"];

				ui.text("Rope 1");
				{
					let bend_model1 = &mut self.m_tuning1.bending_model;
					let mut bend_model1_selected: usize = *bend_model1 as usize;
					if ui.combo_simple_string("Bend Model##1", &mut bend_model1_selected, &bend_models)
					{
						match bend_model1_selected {
							0 => {
								*bend_model1 = B2bendingModel::B2SpringAngleBendingModel;
							}
							1 => {
								*bend_model1 = B2bendingModel::B2PbdAngleBendingModel;
							}
							2 => {
								*bend_model1 = B2bendingModel::B2XpbdAngleBendingModel;
							}
							3 => {
								*bend_model1 = B2bendingModel::B2PbdDistanceBendingModel;
							}
							4 => {
								*bend_model1 = B2bendingModel::B2PbdHeightBendingModel;
							}
							5 => {
								*bend_model1 = B2bendingModel::B2PbdTriangleBendingModel;
							}
							_ => {}
						}
					}					
				}

				Slider::new("Damping##B1", 0.0, 4.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning1.bend_damping);

				Slider::new("Hertz##B1", 0.0, 60.0)
                                .display_format("%.0f")
                                .build(ui, &mut self.m_tuning1.bend_hertz);

				Slider::new("Stiffness##B1", 0.0, 1.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning1.bend_stiffness);

				ui.checkbox(
					"Isometric##1",
					&mut self.m_tuning1.isometric,
				);
				ui.checkbox(
					"Fixed Mass##1",
					&mut self.m_tuning1.fixed_effective_mass,
				);
				ui.checkbox(
					"Warm Start##1",
					&mut self.m_tuning1.warm_start,
				);

				{
					let stretch_model1 = &mut self.m_tuning1.stretching_model;
					let mut stretch_model1_selected: usize = *stretch_model1 as usize;
					if ui.combo_simple_string("Stretch Model##1", &mut stretch_model1_selected, &stretch_models) {
						match stretch_model1_selected {
							0 => {
								*stretch_model1 = B2stretchingModel::B2PbdStretchingModel;
							}
							1 => {
								*stretch_model1 = B2stretchingModel::B2XpbdStretchingModel;
							}
							_ => {}
						}
					}
				}

				Slider::new("Damping##S1", 0.0, 4.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning1.stretch_damping);

				Slider::new("Hertz##S1", 0.0, 60.0)
                                .display_format("%.0f")
                                .build(ui, &mut self.m_tuning1.stretch_hertz);

				Slider::new("Stiffness##S1", 0.0, 1.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning1.stretch_stiffness);

				Slider::new("Iterations##S1", 1, 100)
                                .display_format("%d")
                                .build(ui, &mut self.m_iterations1);

				ui.separator();

				ui.text("Rope 2");
				{
					let bend_model2 = &mut self.m_tuning2.bending_model;
					let mut bend_model2_selected: usize = *bend_model2 as usize;
					if ui.combo_simple_string("Bend Model##2", &mut bend_model2_selected, &bend_models) {
						match bend_model2_selected {
							0 => {
								*bend_model2 = B2bendingModel::B2SpringAngleBendingModel;
							}
							1 => {
								*bend_model2 = B2bendingModel::B2PbdAngleBendingModel;
							}
							2 => {
								*bend_model2 = B2bendingModel::B2XpbdAngleBendingModel;
							}
							3 => {
								*bend_model2 = B2bendingModel::B2PbdDistanceBendingModel;
							}
							4 => {
								*bend_model2 = B2bendingModel::B2PbdHeightBendingModel;
							}
							_ => {}
						}					
					}
				}
				Slider::new("Damping##B2", 0.0, 4.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning2.bend_damping);

				Slider::new("Hertz##B2", 0.0, 60.0)
                                .display_format("%.0f")
                                .build(ui, &mut self.m_tuning2.bend_hertz);

				Slider::new("Stiffness##B2", 0.0, 1.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning2.bend_stiffness);				

				ui.checkbox(
					"Isometric##2",
					&mut self.m_tuning2.isometric,
				);
				ui.checkbox(
					"Fixed Mass##2",
					&mut self.m_tuning2.fixed_effective_mass,
				);
				ui.checkbox(
					"Warm Start##2",
					&mut self.m_tuning2.warm_start,
				);

				{
					let stretch_model2 = &mut self.m_tuning2.stretching_model;
					let mut stretch_model2_selected: usize = *stretch_model2 as usize;
					if ui.combo_simple_string("Stretch Model##2", &mut stretch_model2_selected, &stretch_models) {
					
								match stretch_model2_selected {
									0 => {
										*stretch_model2 = B2stretchingModel::B2PbdStretchingModel;
									}
									1 => {
										*stretch_model2 = B2stretchingModel::B2XpbdStretchingModel;
									}
									_ => {}
								}
							

					}
				}

				Slider::new("Damping##S2", 0.0, 4.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning2.stretch_damping);

				Slider::new("Hertz##S2", 0.0, 60.0)
                                .display_format("%.0f")
                                .build(ui, &mut self.m_tuning2.stretch_hertz);

				Slider::new("Stiffness##S2", 0.0, 1.0)
                                .display_format("%.1f")
                                .build(ui, &mut self.m_tuning2.stretch_stiffness);

				Slider::new("Iterations##S2", 1, 100)
                                .display_format("%d")
                                .build(ui, &mut self.m_iterations2);

				ui.separator();

				Slider::new("Speed", 10.0, 100.0)
                                .display_format("%.0f")
                                .build(ui, &mut self.m_speed);

				if ui.button("Reset") {
					self.m_position1.set(-5.0, 15.0);
					self.m_position2.set(5.0, 15.0);
					self.m_rope1.reset(self.m_position1);
					self.m_rope2.reset(self.m_position2);
				}

				width_token.pop(ui);
			});	
	
		}
	fn keyboard(&mut self, key: &KeyboardInput) {
		match key.virtual_keycode {
			Some(VirtualKeyCode::Comma) => {
				if key.state == ElementState::Pressed {
					self.left_pressed = true;
				}

				if key.state == ElementState::Released {
					self.left_pressed = false;
				}
			}
			Some(VirtualKeyCode::Period) => {
				if key.state == ElementState::Pressed {
					self.right_pressed = true;
				}

				if key.state == ElementState::Released {
					self.right_pressed = false;
				}
			}
			_ => (),
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
		let mut dt: f32 = if settings.m_hertz > 0.0 {
			1.0 / settings.m_hertz
		} else {
			0.0
		};

		if settings.m_pause == true && settings.m_single_step == false {
			dt = 0.0;
		}

		if self.left_pressed {
			self.m_position1.x -= self.m_speed * dt;
			self.m_position2.x -= self.m_speed * dt;
		}

		if self.right_pressed {
			self.m_position1.x += self.m_speed * dt;
			self.m_position2.x += self.m_speed * dt;
		}

		self.m_rope1.set_tuning(&self.m_tuning1);
		self.m_rope2.set_tuning(&self.m_tuning2);
		self.m_rope1.step(dt, self.m_iterations1, self.m_position1);
		self.m_rope2.step(dt, self.m_iterations2, self.m_position2);

		Test::step(self.base.clone(), ui, display, target, settings, *camera);

		{
			let mut base = self.base.borrow_mut();

			self.m_rope1.draw(&mut *base.g_debug_draw.borrow_mut());
			self.m_rope2.draw(&mut *base.g_debug_draw.borrow_mut());

			base.g_debug_draw.borrow().draw_string(
				ui,
				B2vec2::new(5.0, base.m_text_line as f32),
				"Press comma and period to move left and right",
			);
			base.m_text_line += base.m_text_increment;
		}
	}
}

//static int testIndex = RegisterTest("Rope", "Bending", Rope::create);
