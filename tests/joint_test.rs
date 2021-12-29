#[cfg(test)]
mod test {
    use std::cell::RefCell;
    use std::rc::Rc;

    use box2d_rs::b2_body::*;
    use box2d_rs::b2_fixture::*;
    use box2d_rs::b2_math::*;
    use box2d_rs::b2rs_common::UserDataType;
    use box2d_rs::b2_world::*;
    use box2d_rs::shapes::b2_polygon_shape::*;
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
		let bodyC = B2world::create_body(world.clone(), &body_def);

		let mut massData = B2massData::default();
		circle.compute_mass(&mut massData, fixture_def.density);
		let mg: f32 = massData.mass * gravity.y;

		B2body::create_fixture(body_a.clone(), &fixture_def);
		B2body::create_fixture(body_b.clone(), &fixture_def);
		B2body::create_fixture(bodyC.clone(), &fixture_def);

		let mut distanceJointDef = B2distanceJointDef::default();
		distanceJointDef.initialize(ground.clone(), body_a, body_def.position + B2vec2::new(0.0, 4.0), body_def.position);
		distanceJointDef.minLength = distanceJointDef.length;
		distanceJointDef.maxLength = distanceJointDef.length;

		let mut prismaticJointDef = B2prismaticJointDef::default();
		prismaticJointDef.initialize(ground.clone(), body_b, body_def.position, B2vec2::new(1.0, 0.0));

		let mut revoluteJointDef = B2revoluteJointDef::default();
		revoluteJointDef.initialize(ground.clone(), bodyC, body_def.position);

		let distanceJoint = world.borrow_mut().create_joint(&B2JointDefEnum::DistanceJoint(distanceJointDef.clone()));
		let prismaticJoint = world.borrow_mut().create_joint(&B2JointDefEnum::PrismaticJoint(prismaticJointDef.clone()));
		let revoluteJoint = world.borrow_mut().create_joint(&B2JointDefEnum::RevoluteJoint(revoluteJointDef.clone()));

		const timeStep: f32 = 1.0 / 60.;
		const invTimeStep: f32 = 60.0;
		const velocityIterations:i32 = 6;
		const positionIterations:i32 = 2;

		world.borrow_mut().step(timeStep, velocityIterations, positionIterations);

		const tol: f32 = 1.0e-5;
		{
			let F: B2vec2 = distanceJoint.borrow().get_reaction_force(invTimeStep);
			let T: f32 = distanceJoint.borrow().get_reaction_torque(invTimeStep);
			assert!(F.x == 0.0);
			assert!(b2_abs(F.y + mg) < tol);
			assert!(T == 0.0);
		}

		{
			let F: B2vec2 = prismaticJoint.borrow().get_reaction_force(invTimeStep);
			let T: f32 = prismaticJoint.borrow().get_reaction_torque(invTimeStep);
			assert!(F.x == 0.0);
			assert!(b2_abs(F.y + mg) < tol);
			assert!(T == 0.0);
		}

		{
			let F: B2vec2 = revoluteJoint.borrow().get_reaction_force(invTimeStep);
			let T: f32 = revoluteJoint.borrow().get_reaction_torque(invTimeStep);
			assert!(F.x == 0.0);
			assert!(b2_abs(F.y + mg) < tol);
			assert!(T == 0.0);
		}
	}
}
