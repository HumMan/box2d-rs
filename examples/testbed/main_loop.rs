use super::clipboard;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_math::*;
use glium::backend::Facade;
use glium::glutin;
use glium::glutin::dpi::{PhysicalPosition, PhysicalSize};
use glium::glutin::event::{Event, VirtualKeyCode, WindowEvent, MouseButton, ModifiersState, ElementState, MouseScrollDelta};
use glium::glutin::event_loop::{ControlFlow, EventLoop};
use glium::glutin::window::WindowBuilder;
use glium::{Display, Surface};
use imgui::{Context, FontConfig, FontGlyphRanges, FontSource, TabBar, TabItem};
use imgui_glium_renderer::Renderer;
use imgui_winit_support::{HiDpiMode, WinitPlatform};
use std::time::Instant;

use itertools::Itertools;
use std::cell::RefCell;
use std::rc::Rc;

use super::draw::*;
use super::settings::*;
use super::test::{TestEntry, TestPtr};
use super::tests::register::get_tests_list;

#[cfg(feature="serde_support")]
use super::test_serialize;

pub struct System {
    pub event_loop: EventLoop<()>,
    pub display: glium::Display,
    pub imgui: Context,
    pub platform: WinitPlatform,
    pub renderer: Renderer,
    pub font_size: f32,
    pub(crate) s_settings: Settings,
}

pub fn init(title: &str) -> System {
    let title = match title.rfind('/') {
        Some(idx) => title.split_at(idx + 1).1,
        None => title,
    };
    let mut s_settings = Settings::default();
    s_settings.load();
    let event_loop = EventLoop::new();
    let context = glutin::ContextBuilder::new().with_vsync(true);
    let builder = WindowBuilder::new()
        .with_title(title.to_owned())
        .with_inner_size(glutin::dpi::LogicalSize::new(
            s_settings.m_window_width,
            s_settings.m_window_height,
        ));
    let display =
        Display::new(builder, context, &event_loop).expect("Failed to initialize display");

    let mut imgui = Context::create();
    imgui.set_ini_filename(None);

    if let Some(backend) = clipboard::init() {
        imgui.set_clipboard_backend(backend);
    } else {
        eprintln!("Failed to initialize clipboard");
    }

    let mut platform = WinitPlatform::init(&mut imgui);
    {
        let gl_window = display.gl_window();
        let window = gl_window.window();
        // window.set_outer_position(PhysicalPosition{
        //     x: 0, y:0
        // });
        platform.attach_window(imgui.io_mut(), &window, HiDpiMode::Rounded);
    }

    let hidpi_factor = platform.hidpi_factor();
    let font_size = (13.0 * hidpi_factor) as f32;
    imgui.fonts().add_font(&[
        FontSource::DefaultFontData {
            config: Some(FontConfig {
                size_pixels: font_size,
                ..FontConfig::default()
            }),
        },
        FontSource::TtfData {
            data: include_bytes!("../../resources/mplus-1p-regular.ttf"),
            size_pixels: font_size,
            config: Some(FontConfig {
                rasterizer_multiply: 1.75,
                glyph_ranges: FontGlyphRanges::japanese(),
                ..FontConfig::default()
            }),
        },
    ]);

    imgui.io_mut().font_global_scale = (1.0 / hidpi_factor) as f32;

    let renderer = Renderer::init(&mut imgui, &display).expect("Failed to initialize renderer");

    System {
        event_loop,
        display,
        imgui,
        platform,
        renderer,
        font_size,
        s_settings,
    }
}



impl System {
    pub fn main_loop(self) {
        let System {
            event_loop,
            display,
            mut imgui,
            mut platform,
            mut renderer,
            mut s_settings,
            ..
        } = self;
        let mut last_frame = Instant::now();

        let mut g_camera = Camera::default();
        let g_debug_draw = Rc::new(RefCell::new(TestBedDebugDraw::new()));

        let mut s_right_mouse_down = false;
        let mut s_click_point_ws = B2vec2::zero();
        let mut cursor_position: PhysicalPosition<f64> = PhysicalPosition {x:0.0, y:0.0};

        g_debug_draw.borrow_mut().create(&display);

        let g_test_entries = get_tests_list::<glium::Display>();

        let mut s_test;
        s_test =
            (g_test_entries[s_settings.m_test_index as usize].create_fcn)(g_debug_draw.clone());

        Self::resize_callback(
            &PhysicalSize {
                height: s_settings.m_window_height as u32,
                width: s_settings.m_window_width as u32,
            },
            &mut g_camera,
            &mut s_settings,
        );

        let mut settings_saved = false;
        let mut modifiers = ModifiersState::default();

        event_loop.run(move |event, _, control_flow| {
            match &event {
                Event::NewEvents(_) => {
                    let now = Instant::now();
                    imgui
                        .io_mut()
                        .update_delta_time(now.duration_since(last_frame));
                    last_frame = now;
                },
                Event::MainEventsCleared => {
                    let gl_window = display.gl_window();
                    platform
                        .prepare_frame(imgui.io_mut(), &gl_window.window())
                        .expect("Failed to prepare frame");
                    gl_window.window().request_redraw();
                }
                Event::RedrawRequested(_) => {
                    let gl_window = display.gl_window();
                    let mut target = display.draw();

                    {
                        let ui = imgui.frame();
                        target.clear_color(0.2, 0.2, 0.2, 1.0);
                        platform.prepare_render(&ui, gl_window.window());

                        if g_debug_draw.borrow().m_show_ui                        
                        {        
                            s_test.borrow().get_base().borrow_mut().draw_title(
                                &ui,
                                &format!(
                                    "{0} : {1}",
                                    g_test_entries[s_settings.m_test_index as usize].category,
                                    g_test_entries[s_settings.m_test_index as usize].name
                                ),
                            );                            
                        }

                        s_test.borrow_mut().step(
                            &ui,
                            &display,
                            &mut target,
                            &mut s_settings,
                            &mut g_camera,
                        );

                        Self::update_ui(
                            &ui,
                            &g_camera,
                            &mut s_settings,
                            &mut s_test,
                            g_test_entries.clone(),
                            g_debug_draw.clone(),
                            control_flow,
                        );

                        let draw_data = imgui.render();
                        renderer
                            .render(&mut target, draw_data)
                            .expect("Rendering failed");
                        target.finish().expect("Failed to swap buffers");
                    }
                }
                Event::WindowEvent { event, .. } => match event {
                    WindowEvent::Resized(size) => {
                        Self::resize_callback(size, &mut g_camera, &mut s_settings);
                    }
                    WindowEvent::CursorMoved {
                        position, ..
                    } => {
                        cursor_position = *position;
                        Self::mouse_motion_callback(s_test.clone(), &mut g_camera, position, &mut s_right_mouse_down, &mut s_click_point_ws);
                    }
                    WindowEvent::ModifiersChanged(modifiers_updated) => {
                        modifiers = *modifiers_updated;
                    }
                    WindowEvent::MouseInput {
                        button, state, ..
                    } => {
                        Self::mouse_button_callback(s_test.clone(), cursor_position, &mut g_camera, button, state, &modifiers , &mut s_right_mouse_down, &mut s_click_point_ws);
                    }
                    WindowEvent::MouseWheel {
                        delta, ..
                    } => {
                        if !imgui.io().want_capture_mouse
                        {
                            if let MouseScrollDelta::LineDelta(x,y) = delta
                            {
                                Self::scroll_callback(*x,*y,&mut g_camera);
                            }
                        }
                    }
                    WindowEvent::KeyboardInput {
                        input,
                        device_id: _,
                        is_synthetic: _,
                    } => {
                        if input.state == ElementState::Pressed 
                        {
                            match input.virtual_keycode {
                                Some(VirtualKeyCode::Escape) => {
                                    // Quit
                                    *control_flow = ControlFlow::Exit
                                }
                                Some(VirtualKeyCode::Left) => {
                                    // Pan left
                                    if modifiers == ModifiersState::CTRL
                                    {
                                        let new_origin = B2vec2::new(2.0, 0.0);
                                        s_test.borrow_mut().shift_origin(new_origin);
                                    }
                                    else
                                    {
                                        g_camera.m_center.x -= 0.5;
                                    }
                                },
                                Some(VirtualKeyCode::Right) => {
                                    // Pan right
                                    if modifiers == ModifiersState::CTRL
                                    {
                                        let new_origin = B2vec2::new(-2.0, 0.0);
                                        s_test.borrow_mut().shift_origin(new_origin);
                                    }
                                    else
                                    {
                                        g_camera.m_center.x += 0.5;
                                    }
                                },
                                Some(VirtualKeyCode::Down) => {
                                    // Pan down
                                    if modifiers == ModifiersState::CTRL
                                    {
                                        let new_origin = B2vec2::new(0.0, 2.0);
                                        s_test.borrow_mut().shift_origin(new_origin);
                                    }
                                    else
                                    {
                                        g_camera.m_center.y -= 0.5;
                                    }
                                },
                                Some(VirtualKeyCode::Up) => {
                                    // Pan up
                                    if modifiers == ModifiersState::CTRL
                                    {
                                        let new_origin = B2vec2::new(0.0, -2.0);
                                        s_test.borrow_mut().shift_origin(new_origin);
                                    }
                                    else
                                    {
                                        g_camera.m_center.y += 0.5;
                                    }
                                },
                                Some(VirtualKeyCode::Home) => {
                                    // Reset view
                                    g_camera.m_zoom = 1.0;
			                        g_camera.m_center.set(0.0, 20.0);
                                },
                                Some(VirtualKeyCode::Z) => {
                                    // Zoom out
                                    g_camera.m_zoom = b2_max(1.1 * g_camera.m_zoom, 0.02);
                                },
                                Some(VirtualKeyCode::X) => {
                                    // Zoom in
                                    g_camera.m_zoom = b2_max(0.9 * g_camera.m_zoom, 0.02);
                                },
                                Some(VirtualKeyCode::R) => {
                                    // Reset test
                                    s_test = (g_test_entries[s_settings.m_test_index as usize].create_fcn)(
                                        g_debug_draw.clone(),
                                    );
                                },
                                Some(VirtualKeyCode::Space) => {
                                    // Launch a bomb.
                                    s_test.borrow_mut().launch_bomb_rand();
                                },
                                Some(VirtualKeyCode::O) => {
                                    s_settings.m_single_step = !s_settings.m_single_step;
                                },
                                Some(VirtualKeyCode::P) => {
                                    s_settings.m_pause = !s_settings.m_pause;
                                },
                                Some(VirtualKeyCode::LBracket) => {    
                                    // Switch to previous test                                
                                    if s_settings.m_test_index==0 {
                                        s_settings.m_test_index = g_test_entries.len()-1;
                                    }else{
                                        s_settings.m_test_index-=1;
                                    }
                                    s_test = (g_test_entries
                                        [s_settings.m_test_index as usize]
                                        .create_fcn)(
                                        g_debug_draw.clone()
                                    );
                                },
                                Some(VirtualKeyCode::RBracket) => {
                                    // Switch to next test
                                    s_settings.m_test_index+=1;
                                    if s_settings.m_test_index==g_test_entries.len(){
                                        s_settings.m_test_index = 0;
                                    }
                                    s_test = (g_test_entries
                                        [s_settings.m_test_index as usize]
                                        .create_fcn)(
                                        g_debug_draw.clone()
                                    );
                                },
                                Some(VirtualKeyCode::Tab) => {
                                    let mut d = g_debug_draw.borrow_mut();
                                    d.m_show_ui = !d.m_show_ui;
                                },
                                _ => {},
                            }    
                        }      
                        s_test.borrow_mut().keyboard(input);              
                    }
                    WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,
                    _ => (),
                },
                _ => (),
            }
            let gl_window = display.gl_window();
            platform.handle_event(imgui.io_mut(), gl_window.window(), &event);

            if *control_flow == ControlFlow::Exit && !settings_saved {
                settings_saved = true;
                s_settings.save();
            }
        });
    }

    fn scroll_callback(_dx: f32, dy: f32, g_camera: &mut Camera)
    {
        if dy > 0.0
        {
            g_camera.m_zoom /= 1.1;
        }
        else
        {
            g_camera.m_zoom *= 1.1;
        }
    }

    fn mouse_button_callback<'a, D: UserDataType, F: Facade>(
        s_test: TestPtr<D, F>,
        cursor_position: PhysicalPosition<f64>,
        g_camera: &mut Camera,
         button: &MouseButton, 
        action: &ElementState,
         mods: &ModifiersState,
        s_right_mouse_down: &mut bool,
        s_click_point_ws: &mut B2vec2)
    {
        let ps = B2vec2::new(cursor_position.x as f32, cursor_position.y as f32);

        // Use the mouse to move things around.
        if *button == MouseButton::Left
        {
            //<##>
            //ps.set(0, 0);
            let pw: B2vec2 = g_camera.convert_screen_to_world(ps);
            if *action == ElementState::Pressed
            {
                if mods.shift()
                {
                    s_test.borrow_mut().shift_mouse_down(pw);
                }
                else
                {
                    s_test.borrow_mut().mouse_down(pw);
                }
            }
            
            if *action == ElementState::Released
            {
                s_test.borrow_mut().mouse_up(pw);
            }
        }
        else if *button == MouseButton::Right
        {
            if *action == ElementState::Pressed
            {	
                *s_click_point_ws = g_camera.convert_screen_to_world(ps);
                *s_right_mouse_down = true;
            }

            if *action == ElementState::Released
            {
                *s_right_mouse_down = false;
            }
        }
    }

    fn mouse_motion_callback<'a, D: UserDataType, F: Facade>(s_test: TestPtr<D, F>, 
        g_camera: &mut Camera, position: &PhysicalPosition<f64>,
         s_right_mouse_down: &mut bool,
         s_click_point_ws: &mut B2vec2)
    {
        let ps = B2vec2::new(position.x as f32, position.y as f32);

        let pw: B2vec2 = g_camera.convert_screen_to_world(ps);
        s_test.borrow_mut().mouse_move(pw);
        
        if *s_right_mouse_down
        {
            let diff: B2vec2 = pw - *s_click_point_ws;
            g_camera.m_center.x -= diff.x;
            g_camera.m_center.y -= diff.y;
            *s_click_point_ws = g_camera.convert_screen_to_world(ps);
        }
    }

    fn resize_callback(size: &PhysicalSize<u32>, g_camera: &mut Camera, s_settings: &mut Settings) {
        g_camera.m_width = size.width as i32;
        g_camera.m_height = size.height as i32;
        s_settings.m_window_width = size.width as i32;
        s_settings.m_window_height = size.height as i32;
    }

    fn update_ui<'a, D: UserDataType, F: Facade>(
        ui: &imgui::Ui,
        g_camera: &Camera,
        s_settings: &mut Settings,
        s_test: &mut TestPtr<D, F>,
        g_test_entries: Vec<TestEntry<'a, D, F>>,
        g_debug_draw: Rc<RefCell<TestBedDebugDraw>>,
        control_flow: &mut ControlFlow,
    ) {
        let menu_width = 230;

        if g_debug_draw.borrow().m_show_ui
        {

            ui.window("Tools")
                .title_bar(false)
                .flags(
                    imgui::WindowFlags::NO_MOVE
                        | imgui::WindowFlags::NO_RESIZE
                        | imgui::WindowFlags::NO_COLLAPSE,
                )
                .position(
                    [(g_camera.m_width - menu_width) as f32 - 10.0, 10.0],
                    imgui::Condition::Always,
                )
                .size(
                    [menu_width as f32, (g_camera.m_height - 20) as f32],
                    imgui::Condition::Always,
                )
                .build(|| {
                    TabBar::new("ControlTabs").build(ui, || {
                        TabItem::new("Controls").build(ui, || {
                            ui.slider_config("Vel Iters", 0, 50)
                                .display_format("%d")
                                .build(&mut s_settings.m_velocity_iterations);
  
                            ui.slider_config("Pos Iter", 0, 50)
                                .display_format("%d")
                                .build(&mut s_settings.m_position_iterations);

                            ui.slider_config("Hertz", 5.0, 120.0)
                                .display_format("%.0f hz")
                                .build(&mut s_settings.m_hertz);

                            ui.separator();

                            ui.checkbox("Sleep", &mut s_settings.m_enable_sleep);
                            ui.checkbox("Warm Starting", &mut s_settings.m_enable_warm_starting);
                            ui.checkbox("Time of Impact", &mut s_settings.m_enable_continuous);
                            ui.checkbox("Sub-Stepping", &mut s_settings.m_enable_sub_stepping);
     
                            ui.separator();

                            ui.checkbox("Shapes", &mut s_settings.m_draw_shapes);
                            ui.checkbox("Joints", &mut s_settings.m_draw_joints);
                            ui.checkbox("AABBs", &mut s_settings.m_draw_aabbs);
                            ui.checkbox("Contact Points", &mut s_settings.m_draw_contact_points);
                            ui.checkbox("Contact Normals", &mut s_settings.m_draw_contact_normals);
                            ui.checkbox("Contact Impulses", &mut s_settings.m_draw_contact_impulse);
                            ui.checkbox("Friction Impulses", &mut s_settings.m_draw_friction_impulse);
                            ui.checkbox("Center of Masses", &mut s_settings.m_draw_coms);
                            ui.checkbox("Statistics", &mut s_settings.m_draw_stats);
                            ui.checkbox("Profile", &mut s_settings.m_draw_profile);

                            let button_sz = [-1.0, 0.0];
                            if ui.button_with_size("Pause (P)", button_sz) {
                                s_settings.m_pause = !s_settings.m_pause;
                            }
                            if ui.button_with_size("Single Step (O)", button_sz) {
                                s_settings.m_single_step = !s_settings.m_single_step;
                            }
                            if ui.button_with_size("Restart (R)", button_sz) {
                                *s_test = (g_test_entries[s_settings.m_test_index as usize].create_fcn)(
                                    g_debug_draw.clone(),
                                );
                            }
                            #[cfg(feature="serde_support")]
                            {
                                if ui.button_with_size("Serialize", button_sz) {
                                    test_serialize::test_serialize::<D>(
                                        g_test_entries[s_settings.m_test_index as usize].name,
                                        s_test.borrow().get_base().borrow().m_world.clone()
                                    );
                                }
                                if ui.button_with_size("Deserialize", button_sz) {
                                    let new_world = test_serialize::test_deserialize::<D>(g_test_entries[s_settings.m_test_index as usize].name);
                                    let base = s_test.borrow().get_base();
                                    new_world.borrow_mut().set_debug_draw(g_debug_draw.clone());
                                    base.borrow_mut().m_world = new_world;
                                }
                            }
                            if ui.button_with_size("Quit", button_sz) {
                                *control_flow = ControlFlow::Exit;
                            }
                        });

                        let leaf_node_flags: imgui::TreeNodeFlags = imgui::TreeNodeFlags::OPEN_ON_ARROW
                            | imgui::TreeNodeFlags::OPEN_ON_DOUBLE_CLICK
                            | imgui::TreeNodeFlags::LEAF
                            | imgui::TreeNodeFlags::NO_TREE_PUSH_ON_OPEN;
                        let node_flags: imgui::TreeNodeFlags = imgui::TreeNodeFlags::OPEN_ON_ARROW
                            | imgui::TreeNodeFlags::OPEN_ON_DOUBLE_CLICK;

                        TabItem::new("Tests").build(ui, || {
                            let tests = &g_test_entries;

                            for (key, group) in &tests.into_iter().group_by(|v| v.category) {
                                let category_selected: bool =
                                    g_test_entries[s_settings.m_test_index as usize].category == key;

                                ui.tree_node_config(key)
                                    .flags(node_flags)
                                    .selected(category_selected)
                                    .build(|| {
                                        for test in group {
                                            ui.tree_node_config(test.name)
                                                .flags(leaf_node_flags)
                                                .selected(s_settings.m_test_index == test.index)
                                                .build(|| {
                                                    if ui.is_item_clicked_with_button(imgui::MouseButton::Left) {
                                                        s_settings.m_test_index = test.index;
                                                        *s_test = (g_test_entries
                                                            [s_settings.m_test_index as usize]
                                                            .create_fcn)(
                                                            g_debug_draw.clone()
                                                        );
                                                    }
                                                });
                                        }
                                    });
                            }
                        });
                    });
                });

            s_test.borrow_mut().update_ui(&ui);
        }
    }
}
