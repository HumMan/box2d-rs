#[cfg(test)]
mod math_test {
    use box2d_rs::b2_math::*;
    use box2d_rs::b2_settings::*;

    #[derive(Default, Copy, Clone, Debug,PartialEq)]
    struct FixtureData {
        id: i32,
    }

    #[derive(Default, Copy, Clone, Debug, PartialEq)]
    struct UserDataTypes;
    impl UserDataType for UserDataTypes {
        type Fixture = FixtureData;
        type Body = FixtureData;
        type Joint = FixtureData;
    }

    #[test]
    fn sweep() {
		// From issue #447
		let mut sweep = B2Sweep::default();
		sweep.local_center.set_zero();
		sweep.c0.set(3.0, 4.0);
		sweep.c.set(3.0, 4.0);
		sweep.a0 = 0.0;
		sweep.a = 0.0;
		sweep.alpha0 = 0.0;

		let mut transform = B2Transform::default();
		sweep.get_transform(&mut transform, 0.6);

		assert_eq!(transform.p.x, sweep.c0.x);
		assert_eq!(transform.p.y, sweep.c0.y);
	}
}
