use super::super::draw::*;
use super::super::settings::*;
use super::super::test::*;
use box2d_rs::b2_body::*;
use box2d_rs::b2_collision::*;
use box2d_rs::b2_contact::*;
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

struct B2contactListenerCustom<D: UserDataType> {
    base: B2testContactListenerDefault<D>,
    test_data: Rc<RefCell<TestData<D>>>,
}

impl<D: UserDataType> B2contactListener<D> for B2contactListenerCustom<D> {
    fn begin_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
        self.base.begin_contact(contact);
    }
    fn end_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
        self.base.end_contact(contact);
    }
    fn pre_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, old_manifold: &B2manifold) {
        self.base.pre_solve(contact, old_manifold);
        let test_data = self.test_data.borrow();

        let fixture_a = contact.get_base().get_fixture_a();
        let fixture_b = contact.get_base().get_fixture_b();

        if Rc::ptr_eq(&fixture_a, test_data.m_platform.as_ref().unwrap()) {
            contact.get_base_mut().set_tangent_speed(5.0);
        }

        if Rc::ptr_eq(&fixture_b, test_data.m_platform.as_ref().unwrap()) {
            contact.get_base_mut().set_tangent_speed(-5.0);
        }
    }
    fn post_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, impulse: &B2contactImpulse) {
        self.base.post_solve(contact, impulse);
    }
}

#[derive(Default)]
pub(crate) struct TestData<D: UserDataType> {
    pub(crate) m_platform: Option<FixturePtr<D>>,
    pub(crate) m_world: Option<B2worldPtr<D>>,
}

pub(crate) struct ConveyorBelt<D: UserDataType> {
    base: TestBasePtr<D>,
    destruction_listener: B2destructionListenerPtr<D>,
    contact_listener: Option<B2contactListenerPtr<D>>,
    test_data: Rc<RefCell<TestData<D>>>,
}

impl<D: UserDataType> ConveyorBelt<D> {
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

        // Ground
        {
            let bd = B2bodyDef::default();
            let ground = B2world::create_body(m_world.clone(), &bd);

            let mut shape = B2edgeShape::default();
            shape.set_two_sided(B2vec2::new(-20.0, 0.0), B2vec2::new(20.0, 0.0));
            B2body::create_fixture_by_shape(ground, Rc::new(RefCell::new(shape)), 0.0);
        }

        // Platform
        {
            let mut bd = B2bodyDef::default();
            bd.position.set(-5.0, 5.0);
            let body = B2world::create_body(m_world.clone(), &bd);

            let mut shape = B2polygonShape::default();
            shape.set_as_box(10.0, 0.5);

            let mut fd = B2fixtureDef::default();
            fd.shape = Some(Rc::new(RefCell::new(shape)));
            fd.friction = 0.8;
            test_data.m_platform = Some(B2body::create_fixture(body.clone(), &fd));
        }

        // Boxes
        for i in 0..5 {
            let mut bd = B2bodyDef::default();
            bd.body_type = B2bodyType::B2DynamicBody;
            bd.position.set(-10.0 + 2.0 * i as f32, 7.0);
            let body = B2world::create_body(m_world.clone(), &bd);

            let mut shape = B2polygonShape::default();
            shape.set_as_box(0.5, 0.5);
            B2body::create_fixture_by_shape(body, Rc::new(RefCell::new(shape)), 20.0);
        }
    }
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for ConveyorBelt<D> {
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
