use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_rope::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2_world_callbacks::*;

use std::ffi::CString;

use imgui::im_str;
use imgui::sys;

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
		self.m_tuning1.bending_model = B2bendingModel::B2XpbdAngleBendingModel;
		self.m_tuning1.isometric = true;

		self.m_tuning1.stretch_hertz = 30.0;
		self.m_tuning1.stretch_damping = 4.0;
		self.m_tuning1.stretch_stiffness = 1.0;
		self.m_tuning1.stretching_model = B2stretchingModel::B2XpbdStretchingModel;

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
		imgui::Window::new(im_str!("Tuning"))
			.flags(imgui::WindowFlags::NO_MOVE | imgui::WindowFlags::NO_RESIZE)
			.position([10.0, 100.0], imgui::Condition::Always)
			.size([200.0, 700.0], imgui::Condition::Always)
			.build(&ui, || unsafe {
				sys::igSeparator();

				sys::igPushItemWidth(sys::igGetWindowWidth() * 0.5);
				const COMBO_FLAGS: sys::ImGuiComboFlags = 0;
				let bend_models = ["Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height"];
				let stretch_models = ["PBD", "XPBD"];

				sys::igText(im_str!("Rope 1").as_ptr());
				{
					let bend_model1 = &mut self.m_tuning1.bending_model;
					let bend_model1_name =
						CString::new(bend_models[*bend_model1 as usize]).unwrap();
					if sys::igBeginCombo(
						im_str!("Bend Model##1").as_ptr(),
						bend_model1_name.as_ptr(),
						COMBO_FLAGS,
					) {
						for (i, b) in bend_models.iter().enumerate() {
							let is_selected: bool = *bend_model1 as usize == i;
							let b_name = CString::new(*b).unwrap();
							if sys::igSelectable(
								b_name.as_ptr(),
								is_selected,
								0,
								sys::ImVec2 { x: 0.0, y: 0.0 },
							) {
								match i {
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
									_ => {}
								}
							}

							if is_selected {
								sys::igSetItemDefaultFocus();
							}
						}
						sys::igEndCombo();
					}
				}

				sys::igSliderFloat(
					im_str!("Damping##B1").as_ptr(),
					&mut self.m_tuning1.bend_damping,
					0.0,
					4.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Hertz##B1").as_ptr(),
					&mut self.m_tuning1.bend_hertz,
					0.0,
					60.0,
					im_str!("%.0f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Stiffness##B1").as_ptr(),
					&mut self.m_tuning1.bend_stiffness,
					0.0,
					1.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);

				sys::igCheckbox(
					im_str!("Isometric##1").as_ptr(),
					&mut self.m_tuning1.isometric,
				);
				sys::igCheckbox(
					im_str!("Fixed Mass##1").as_ptr(),
					&mut self.m_tuning1.fixed_effective_mass,
				);
				sys::igCheckbox(
					im_str!("Warm Start##1").as_ptr(),
					&mut self.m_tuning1.warm_start,
				);

				{
					let stretch_model1 = &mut self.m_tuning1.stretching_model;
					let stretch_model1_name =
						CString::new(stretch_models[*stretch_model1 as usize]).unwrap();
					if sys::igBeginCombo(
						im_str!("Stretch Model##1").as_ptr(),
						stretch_model1_name.as_ptr(),
						COMBO_FLAGS,
					) {
						for (i, s) in stretch_models.iter().enumerate() {
							let is_selected: bool = *stretch_model1 as usize == i;
							let s_name = CString::new(*s).unwrap();
							if sys::igSelectable(
								s_name.as_ptr(),
								is_selected,
								0,
								sys::ImVec2 { x: 0.0, y: 0.0 },
							) {
								match i {
									0 => {
										*stretch_model1 = B2stretchingModel::B2PbdStretchingModel;
									}
									1 => {
										*stretch_model1 = B2stretchingModel::B2XpbdStretchingModel;
									}
									_ => {}
								}
							}

							if is_selected {
								sys::igSetItemDefaultFocus();
							}
						}
						sys::igEndCombo();
					}
				}

				sys::igSliderFloat(
					im_str!("Damping##S1").as_ptr(),
					&mut self.m_tuning1.stretch_damping,
					0.0,
					4.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Hertz##S1").as_ptr(),
					&mut self.m_tuning1.stretch_hertz,
					0.0,
					60.0,
					im_str!("%.0f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Stiffness##S1").as_ptr(),
					&mut self.m_tuning1.stretch_stiffness,
					0.0,
					1.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);

				sys::igSliderInt(
					im_str!("Iterations##1").as_ptr(),
					&mut self.m_iterations1,
					1,
					100,
					im_str!("%d").as_ptr(),
				);

				sys::igSeparator();

				sys::igText(im_str!("Rope 2").as_ptr());
				{
					let bend_model2 = &mut self.m_tuning2.bending_model;
					let bend_model2_name =
						CString::new(bend_models[*bend_model2 as usize]).unwrap();
					if sys::igBeginCombo(
						im_str!("Bend Model##2").as_ptr(),
						bend_model2_name.as_ptr(),
						COMBO_FLAGS,
					) {
						for (i, b) in bend_models.iter().enumerate() {
							let is_selected: bool = *bend_model2 as usize == i;
							let b_name = CString::new(*b).unwrap();
							if sys::igSelectable(
								b_name.as_ptr(),
								is_selected,
								0,
								sys::ImVec2 { x: 0.0, y: 0.0 },
							) {
								match i {
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

							if is_selected {
								sys::igSetItemDefaultFocus();
							}
						}
						sys::igEndCombo();
					}
				}

				sys::igSliderFloat(
					im_str!("Damping##B2").as_ptr(),
					&mut self.m_tuning2.bend_damping,
					0.0,
					4.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Hertz##B2").as_ptr(),
					&mut self.m_tuning2.bend_hertz,
					0.0,
					60.0,
					im_str!("%.0f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Stiffness##B2").as_ptr(),
					&mut self.m_tuning2.bend_stiffness,
					0.0,
					1.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);

				sys::igCheckbox(
					im_str!("Isometric##2").as_ptr(),
					&mut self.m_tuning2.isometric,
				);
				sys::igCheckbox(
					im_str!("Fixed Mass##2").as_ptr(),
					&mut self.m_tuning2.fixed_effective_mass,
				);
				sys::igCheckbox(
					im_str!("Warm Start##2").as_ptr(),
					&mut self.m_tuning2.warm_start,
				);

				{
					let stretch_model2 = &mut self.m_tuning2.stretching_model;
					let stretch_model2_name =
						CString::new(stretch_models[*stretch_model2 as usize]).unwrap();
					if sys::igBeginCombo(
						im_str!("Stretch Model##2").as_ptr(),
						stretch_model2_name.as_ptr(),
						COMBO_FLAGS,
					) {
						for (i, s) in stretch_models.iter().enumerate() {
							let is_selected: bool = *stretch_model2 as usize == i;
							let s_name = CString::new(*s).unwrap();
							if sys::igSelectable(
								s_name.as_ptr(),
								is_selected,
								0,
								sys::ImVec2 { x: 0.0, y: 0.0 },
							) {
								match i {
									0 => {
										*stretch_model2 = B2stretchingModel::B2PbdStretchingModel;
									}
									1 => {
										*stretch_model2 = B2stretchingModel::B2XpbdStretchingModel;
									}
									_ => {}
								}
							}

							if is_selected {
								sys::igSetItemDefaultFocus();
							}
						}
						sys::igEndCombo();
					}
				}

				sys::igSliderFloat(
					im_str!("Damping##S2").as_ptr(),
					&mut self.m_tuning2.stretch_damping,
					0.0,
					4.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Hertz##S2").as_ptr(),
					&mut self.m_tuning2.stretch_hertz,
					0.0,
					60.0,
					im_str!("%.0f").as_ptr(),
					1.0,
				);
				sys::igSliderFloat(
					im_str!("Stiffness##S2").as_ptr(),
					&mut self.m_tuning2.stretch_stiffness,
					0.0,
					1.0,
					im_str!("%.1f").as_ptr(),
					1.0,
				);

				sys::igSliderInt(
					im_str!("Iterations##2").as_ptr(),
					&mut self.m_iterations2,
					1,
					100,
					im_str!("%d").as_ptr(),
				);

				sys::igSeparator();

				sys::igSliderFloat(
					im_str!("Speed").as_ptr(),
					&mut self.m_speed,
					10.0,
					100.0,
					im_str!("%.0f").as_ptr(),
					1.0,
				);

				if sys::igButton(im_str!("Reset").as_ptr(), sys::ImVec2 { x: 0.0, y: 0.0 }) {
					self.m_position1.set(-5.0, 15.0);
					self.m_position2.set(5.0, 15.0);
					self.m_rope1.reset(self.m_position1);
					self.m_rope2.reset(self.m_position2);
				}

				sys::igPopItemWidth();
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
