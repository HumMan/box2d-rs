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
use box2d_rs::shapes::b2_edge_shape::*;
use box2d_rs::shapes::b2_polygon_shape::*;

use glium::backend::Facade;
use std::cell::RefCell;
use std::rc::Rc;

// This is used to test sensor shapes.
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
    }
    fn post_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, impulse: &B2contactImpulse) {
        let mut test = self.test_data.borrow_mut();
        if test.m_broke {
            // The body already broke.
            return;
        }

        // Should the body break?
        let count = contact.get_base().get_manifold().point_count;

        let mut max_impulse: f32 = 0.0;
        for i in 0..count {
            max_impulse = b2_max(max_impulse, impulse.normal_impulses[i]);
        }

        if max_impulse > 40.0 {
            // Flag the body for breaking.
            test.m_break = true;
        }
    }
}

#[derive(Default)]
pub(crate) struct TestData<D: UserDataType> {
    pub(crate) m_body1: Option<BodyPtr<D>>,
    pub(crate) m_velocity: B2vec2,
    pub(crate) m_angular_velocity: f32,
    pub(crate) m_shape1: B2polygonShape,
    pub(crate) m_shape2: B2polygonShape,
    pub(crate) m_piece1: Option<FixturePtr<D>>,
    pub(crate) m_piece2: Option<FixturePtr<D>>,

    pub(crate) m_broke: bool,
    pub(crate) m_break: bool,
    pub(crate) m_world: Option<B2worldPtr<D>>,
}

pub(crate) struct Breakable<D: UserDataType> {
    base: TestBasePtr<D>,
    destruction_listener: B2destructionListenerPtr<D>,
    contact_listener: Option<B2contactListenerPtr<D>>,
    test_data: Rc<RefCell<TestData<D>>>,
}

impl<D: UserDataType> Breakable<D> {

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

        // Ground body
        {
            let bd = B2bodyDef::default();
            let ground = B2world::create_body(m_world.clone(), &bd);

            let mut shape = B2edgeShape::default();
            shape.set_two_sided(B2vec2::new(-40.0, 0.0), B2vec2::new(50.0, 0.0));

            B2body::create_fixture_by_shape(ground.clone(), Rc::new(RefCell::new(shape)), 0.0);
        }

        // Breakable dynamic body
        {
            let mut bd = B2bodyDef::default();
            bd.body_type = B2bodyType::B2DynamicBody;
            bd.position.set(0.0, 40.0);
            bd.angle = 0.25 * B2_PI;
            test_data.m_body1 = Some(B2world::create_body(m_world.clone(), &bd));

            test_data
                .m_shape1
                .set_as_box_angle(0.5, 0.5, B2vec2::new(-0.5, 0.0), 0.0);
            test_data.m_piece1 = Some(B2body::create_fixture_by_shape(
                test_data.m_body1.clone().unwrap(),
                Rc::new(RefCell::new(test_data.m_shape1)),
                1.0,
            ));

            test_data
                .m_shape2
                .set_as_box_angle(0.5, 0.5, B2vec2::new(0.5, 0.0), 0.0);
            test_data.m_piece2 = Some(B2body::create_fixture_by_shape(
                test_data.m_body1.clone().unwrap(),
                Rc::new(RefCell::new(test_data.m_shape2)),
                1.0,
            ));
        }

        test_data.m_break = false;
        test_data.m_broke = false;
    }

    fn break_body(test_data: &mut TestData<D>) {
        let m_world = test_data.m_world.clone().unwrap();

        // create two bodies from one.
        let body1 = test_data.m_piece1.as_ref().unwrap().borrow().get_body();
        let center: B2vec2 = body1.borrow().get_world_center();

        B2body::destroy_fixture(body1.clone(), test_data.m_piece2.clone().unwrap());

        test_data.m_piece2 = None;

        let mut bd = B2bodyDef::default();
        bd.body_type = B2bodyType::B2DynamicBody;
        bd.position = body1.borrow().get_position();
        bd.angle = body1.borrow().get_angle();

        let body2 = B2world::create_body(m_world, &bd);
        test_data.m_piece2 = Some(B2body::create_fixture_by_shape(
            body2.clone(),
            Rc::new(RefCell::new(test_data.m_shape2)),
            1.0,
        ));

        // Compute consistent velocities for new bodies based on
        // cached velocity.
        let center1: B2vec2 = body1.borrow().get_world_center();
        let center2: B2vec2 = body2.borrow().get_world_center();

        let velocity1: B2vec2 = test_data.m_velocity
            + b2_cross_scalar_by_vec(test_data.m_angular_velocity, center1 - center);
        let velocity2: B2vec2 = test_data.m_velocity
            + b2_cross_scalar_by_vec(test_data.m_angular_velocity, center2 - center);

        body1
            .borrow_mut()
            .set_angular_velocity(test_data.m_angular_velocity);
        body1.borrow_mut().set_linear_velocity(velocity1);

        body2
            .borrow_mut()
            .set_angular_velocity(test_data.m_angular_velocity);
        body2.borrow_mut().set_linear_velocity(velocity2);
    }
}

impl<D: UserDataType, F: Facade> TestDyn<D, F> for Breakable<D> {
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
        {
            let test_data = self.test_data.clone();
            let mut test_data = test_data.borrow_mut();
            if test_data.m_break {
                Breakable::break_body(&mut test_data);
                test_data.m_broke = true;
                test_data.m_break = false;
            }

            // Cache velocities to improve movement on breakage.
            if test_data.m_broke == false {
                let m_body1 = test_data.m_body1.clone().unwrap();
                let m_body1 = m_body1.borrow();
                test_data.m_velocity = m_body1.get_linear_velocity();
                test_data.m_angular_velocity = m_body1.get_angular_velocity();
            }
        }

        Test::step(self.base.clone(), ui, display, target, settings, *camera);
    }
}
