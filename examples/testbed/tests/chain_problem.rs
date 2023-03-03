use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_fixture::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2rs_common::UserDataType;
use box2d_rs::b2_world::*;
use box2d_rs::b2_world_callbacks::*;
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

pub(crate) struct ChainProblem<D: UserDataType> {
    base: TestBasePtr<D>,
    destruction_listener: B2destructionListenerPtr<D>,
    contact_listener: B2contactListenerPtr<D>,
}

impl<D: UserDataType> ChainProblem<D> {
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
            let g = B2vec2::new(0.0, -10.0);
            m_world.borrow_mut().set_gravity(g);
            let mut bodies = <[Option<BodyPtr<D>>; 2]>::default();
            //let joints = <[Option<B2jointPtr<D>>;2]>::default();
            {
                let mut bd = B2bodyDef::default();
                bd.body_type = B2bodyType::B2StaticBody;
                bodies[0] = Some(B2world::create_body(m_world.clone(), &bd));

                {
                    //let mut fd = B2fixtureDef::default();

                    let v1 = B2vec2::new(0.0, 1.0);
                    let v2 = B2vec2::new(0.0, 0.0);
                    let v3 = B2vec2::new(4.0, 0.0);

                    let mut shape = B2edgeShape::default();
                    shape.set_two_sided(v1, v2);
                    B2body::create_fixture_by_shape(
                        bodies[0].clone().unwrap(),
                        Rc::new(RefCell::new(shape)),
                        0.0,
                    );

                    shape.set_two_sided(v2, v3);
                    B2body::create_fixture_by_shape(
                        bodies[0].clone().unwrap(),
                        Rc::new(RefCell::new(shape)),
                        0.0,
                    );
                }
            }
            {
                let mut bd = B2bodyDef::default();
                bd.body_type = B2bodyType::B2DynamicBody;
                //bd.position.set(6.033980250358582e-01f, 3.028350114822388e+00f);
                bd.position.set(1.0, 3.0);
                bodies[1] = Some(B2world::create_body(m_world.clone(), &bd));

                {
                    let mut fd = B2fixtureDef::default();
                    fd.friction = 0.2;
                    fd.density = 10.0;
                    let mut shape = B2polygonShape::default();
                    let vs: [B2vec2; 4] = [
                        B2vec2::new(0.5, -3.0),
                        B2vec2::new(0.5, 3.0),
                        B2vec2::new(-0.5, 3.0),
                        B2vec2::new(-0.5, -3.0),
                    ];
                    shape.set(&vs);

                    fd.shape = Some(Rc::new(RefCell::new(shape)));

                    B2body::create_fixture(bodies[1].clone().unwrap(), &fd);
                }
            }
        }
    }
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for ChainProblem<D> {
    fn get_base(&self) -> TestBasePtr<D> {
        return self.base.clone();
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
    }
}
