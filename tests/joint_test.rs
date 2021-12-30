#[cfg(test)]
mod test {
    use std::cell::RefCell;
    use std::rc::Rc;

    use box2d_rs::b2_body::*;
    use box2d_rs::b2_fixture::*;
    use box2d_rs::b2_math::*;
    use box2d_rs::b2rs_common::UserDataType;
    use box2d_rs::b2_world::*;
	use box2d_rs::shapes::b2_circle_shape::*;
	use box2d_rs::b2_shape::*;
	use box2d_rs::b2_joint::*;
	use box2d_rs::joints::b2_distance_joint::*;
	use box2d_rs::joints::b2_prismatic_joint::*;
	use box2d_rs::joints::b2_revolute_joint::*;

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

    #[test]
	fn joint_reactions() {
		let gravity = B2vec2::new(0.0, -10.0);
		let world = B2world::<UserDataTypes>::new(gravity);

		let mut body_def = B2bodyDef::default();
		let ground = B2world::create_body(world.clone(), &body_def);

		let mut circle = B2circleShape::default();
		circle.base.m_radius = 1.0;

		let mut fixture_def = B2fixtureDef::default();

		// Disable collision
		fixture_def.filter.mask_bits = 0;
		fixture_def.density = 1.0;
		fixture_def.shape = Some(Rc::new(RefCell::new(circle)));

		body_def.body_type = B2bodyType::B2DynamicBody;
		body_def.position.set(-2.0, 3.0);

		let body_a = B2world::create_body(world.clone(), &body_def);
		let body_b = B2world::create_body(world.clone(), &body_def);
		let body_c = B2world::create_body(world.clone(), &body_def);

		let mut mass_data = B2massData::default();
		circle.compute_mass(&mut mass_data, fixture_def.density);
		let mg: f32 = mass_data.mass * gravity.y;

		B2body::create_fixture(body_a.clone(), &fixture_def);
		B2body::create_fixture(body_b.clone(), &fixture_def);
		B2body::create_fixture(body_c.clone(), &fixture_def);

		let mut distance_joint_def = B2distanceJointDef::default();
		distance_joint_def.initialize(ground.clone(), body_a, body_def.position + B2vec2::new(0.0, 4.0), body_def.position);
		distance_joint_def.min_length = distance_joint_def.length;
		distance_joint_def.max_length = distance_joint_def.length;

		let mut prismatic_joint_def = B2prismaticJointDef::default();
		prismatic_joint_def.initialize(ground.clone(), body_b, body_def.position, B2vec2::new(1.0, 0.0));

		let mut revolute_joint_def = B2revoluteJointDef::default();
		revolute_joint_def.initialize(ground.clone(), body_c, body_def.position);

		let distance_joint = world.borrow_mut().create_joint(&B2JointDefEnum::DistanceJoint(distance_joint_def.clone()));
		let prismatic_joint = world.borrow_mut().create_joint(&B2JointDefEnum::PrismaticJoint(prismatic_joint_def.clone()));
		let revolute_joint = world.borrow_mut().create_joint(&B2JointDefEnum::RevoluteJoint(revolute_joint_def.clone()));

		const TIME_STEP: f32 = 1.0 / 60.;
		const INV_TIME_STEP: f32 = 60.0;
		const VELOCITY_ITERATIONS:i32 = 6;
		const POSITION_ITERATIONS:i32 = 2;

		world.borrow_mut().step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);

		const TOL: f32 = 1.0e-5;
		{
			let f: B2vec2 = distance_joint.borrow().get_reaction_force(INV_TIME_STEP);
			let t: f32 = distance_joint.borrow().get_reaction_torque(INV_TIME_STEP);
			assert!(f.x == 0.0);
			assert!(b2_abs(f.y + mg) < TOL);
			assert!(t == 0.0);
		}

		{
			let f: B2vec2 = prismatic_joint.borrow().get_reaction_force(INV_TIME_STEP);
			let t: f32 = prismatic_joint.borrow().get_reaction_torque(INV_TIME_STEP);
			assert!(f.x == 0.0);
			assert!(b2_abs(f.y + mg) < TOL);
			assert!(t == 0.0);
		}

		{
			let f: B2vec2 = revolute_joint.borrow().get_reaction_force(INV_TIME_STEP);
			let t: f32 = revolute_joint.borrow().get_reaction_torque(INV_TIME_STEP);
			assert!(f.x == 0.0);
			assert!(b2_abs(f.y + mg) < TOL);
			assert!(t == 0.0);
		}
	}
}
