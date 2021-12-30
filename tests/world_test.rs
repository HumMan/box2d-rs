#[cfg(test)]
mod test {
    use std::cell::RefCell;
    use std::rc::Rc;

    use box2d_rs::b2_body::*;
    use box2d_rs::b2_math::*;
    use box2d_rs::b2rs_common::UserDataType;
    use box2d_rs::b2_world::*;
	use box2d_rs::shapes::b2_circle_shape::*;
	use box2d_rs::b2_world_callbacks::*;
	use box2d_rs::b2_contact::*;
	use box2d_rs::b2_collision::*;
	use box2d_rs::b2_common::*;

    #[cfg(feature="serde_support")]
	use serde::{Serialize, Deserialize};

    #[derive(Default, Copy, Clone, Debug, PartialEq)]
    #[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
    struct UserDataTypes;
    impl UserDataType for UserDataTypes {
        type Fixture = i32;
        type Body = i32;
        type Joint = i32;
    }

	pub(crate)struct MyContactListener
	{
		pub begin_contact:bool
	}
	impl<D: UserDataType> B2contactListener<D> for MyContactListener {
		fn begin_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
			b2_not_used(contact);
			self.begin_contact = true;
		}
		fn end_contact(&mut self, contact: &mut dyn B2contactDynTrait<D>) {
			b2_not_used(contact);
		}
		fn pre_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, old_manifold: &B2manifold) {
			b2_not_used(contact);
			b2_not_used(old_manifold);
		}
		fn post_solve(&mut self, contact: &mut dyn B2contactDynTrait<D>, impulse: &B2contactImpulse) {
			b2_not_used(contact);
			b2_not_used(impulse);
		}
	}

	#[test]
	fn begin_contact()
	{
		let world = B2world::<UserDataTypes>::new(B2vec2::new(0.0, -10.0));
		let listener = Rc::new(RefCell::new(MyContactListener{
			begin_contact:false
		}));
		world.borrow_mut().set_contact_listener(listener.clone());

		let mut circle = B2circleShape::default();
		circle.base.m_radius = 5.0;

		let mut body_def = B2bodyDef::default();
		body_def.body_type = B2bodyType::B2DynamicBody;

		let body_a = B2world::create_body(world.clone(), &body_def);
		let body_b = B2world::create_body(world.clone(), &body_def);
		B2body::create_fixture_by_shape(body_a.clone(),Rc::new(RefCell::new(circle)), 0.0);
		B2body::create_fixture_by_shape(body_b.clone(),Rc::new(RefCell::new(circle)), 0.0);

		body_a.borrow_mut().set_transform(B2vec2::new(0.0, 0.0), 0.0);
		body_b.borrow_mut().set_transform(B2vec2::new(100.0, 0.0), 0.0);

		const TIME_STEP: f32 = 1.0 / 60.0;
		const VELOCITY_ITERATIONS:i32 = 6;
		const POSITION_ITERATIONS:i32 = 2;

		world.borrow_mut().step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);

		assert_eq!(world.borrow().get_contact_list().iter().next().is_none(), true);
		assert_eq!(listener.borrow().begin_contact, false);
		
		body_b.borrow_mut().set_transform(B2vec2::new(1.0, 0.0), 0.0);

		world.borrow_mut().step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);

		assert_eq!(world.borrow().get_contact_list().iter().next().is_none(), false);
		assert_eq!(listener.borrow().begin_contact, true);
	}
}