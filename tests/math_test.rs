#[cfg(test)]
mod math_test {
    use box2d_rs::b2_math::*;
    //use box2d_rs::b2rs_common::UserDataType;

    // #[cfg(feature="serde_support")]
	// use serde::{Serialize, Deserialize};

    // #[derive(Default, Copy, Clone, Debug,PartialEq)]
    // #[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
    // struct FixtureData {
    //     id: i32,
    // }

    // #[derive(Default, Copy, Clone, Debug, PartialEq)]
    // #[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
    // struct UserDataTypes;
    // impl UserDataType for UserDataTypes {
    //     type Fixture = FixtureData;
    //     type Body = FixtureData;
    //     type Joint = FixtureData;
    // }

    #[test]
    fn sweep() {
		// From issue #447
		let mut sweep = B2Sweep::default();
        sweep.local_center.set_zero();
		sweep.c0.set(-2.0, 4.0);
		sweep.c.set(3.0, 8.0);
		sweep.a0 = 0.5;
		sweep.a = 5.0;

		let mut transform = B2Transform::default();
		sweep.get_transform(&mut transform, 0.0);

		assert_eq!(transform.p.x, sweep.c0.x);
		assert_eq!(transform.p.y, sweep.c0.y);

        assert_eq!(transform.q.c, f32::cos(sweep.a0));
		assert_eq!(transform.q.s, f32::sin(sweep.a0));

		sweep.get_transform(&mut transform, 1.0);
		assert_eq!(transform.p.x, sweep.c.x);
		assert_eq!(transform.p.y, sweep.c.y);
		assert_eq!(transform.q.c, f32::cos(sweep.a));
		assert_eq!(transform.q.s, f32::sin(sweep.a));

	}
}
