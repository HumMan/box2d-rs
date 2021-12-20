use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_settings::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_circle_shape::*;
use box2d_rs::shapes::b2_edge_shape::*;

use glium::backend::Facade;
use glium::glutin::event::{ElementState, KeyboardInput, VirtualKeyCode};
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct Heavy2<D: UserDataType> {
    base: TestBasePtr<D>,
    destruction_listener: B2destructionListenerPtr<D>,
    contact_listener: B2contactListenerPtr<D>,
    m_heavy: Option<BodyPtr<D>>,
}

impl<D: UserDataType> Heavy2<D> {
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
            m_heavy: None,
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

        {
            let bd = B2bodyDef::default();
            let ground = B2world::create_body(m_world.clone(), &bd);

            let mut shape = B2edgeShape::default();
            shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(40.0, 0.0));
            B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
        }

        let mut bd = B2bodyDef::default();
        bd.body_type = B2bodyType::B2DynamicBody;
        bd.position.set(0.0, 2.5);
        let body = B2world::create_body(m_world.clone(), &bd);

        let mut shape = B2circleShape::default();
        shape.base.m_radius = 0.5;
        B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 10.0);
        bd.position.set(0.0, 3.5);
        let body = B2world::create_body(m_world.clone(), &bd);
        B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 10.0);
    }

    fn toggle_heavy(&mut self) {
        let m_world = self.base.borrow().m_world.clone();

        if let Some(m_heavy) = self.m_heavy.take() {
            m_world.borrow_mut().destroy_body(m_heavy);
        } else {
            let mut bd = B2bodyDef::default();
            bd.body_type = B2bodyType::B2DynamicBody;
            bd.position.set(0.0, 9.0);
            let m_heavy = B2world::create_body(m_world.clone(), &bd);
            self.m_heavy = Some(m_heavy.clone());
            let mut shape = B2circleShape::default();
            shape.base.m_radius = 5.0;
            B2body::create_fixture_by_shape(m_heavy, Rc::new(RefCell::new(shape)), 10.0);
        }
    }
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Heavy2<D> {
    fn get_base(&self) -> TestBasePtr<D> {
        return self.base.clone();
    }
    fn keyboard(&mut self, key: &KeyboardInput) {
        if key.state == ElementState::Pressed {
            match key.virtual_keycode {
                Some(VirtualKeyCode::H) => {
                    self.toggle_heavy();
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
    }
}
