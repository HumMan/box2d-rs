/// Unit tests for collision algorithms
#[cfg(test)]
mod collision_test {
	use box2d_rs::b2_math::*;
	use box2d_rs::b2_shape::*;
    use box2d_rs::b2_settings::*;
    use box2d_rs::shapes::b2_polygon_shape::*;

	#[cfg(feature="serde_support")]
	use serde::{Serialize, Deserialize};

    #[derive(Default, Copy, Clone, Debug, PartialEq)]
	#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
    struct FixtureData {
        id: i32,
    }

    #[derive(Default, Copy, Clone, Debug, PartialEq)]
	#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
    struct UserDataTypes;
    impl UserDataType for UserDataTypes {
        type Fixture = FixtureData;
        type Body = FixtureData;
        type Joint = FixtureData;
    }

#[test]
    fn polygon_mass_data() {
		let center = B2vec2::new(100.0, -50.0);
		let hx: f32 = 0.5;
		let hy: f32 = 1.5;
		let angle1: f32 = 0.25;

		// Data from issue #422. Not used because the data exceeds accuracy limits.
		//constlet center = B2vec2::new(-15000.0f, -15000.0);
		//const float hx = 0.72f, hy = 0.72f;
		//const float angle1 = 0.0f;

		let mut polygon1 = B2polygonShape::default();
		polygon1.set_as_box_angle(hx, hy, center, angle1);

		let abs_tol: f32 = 2.0 * B2_EPSILON;
		let rel_tol: f32 = 2.0 * B2_EPSILON;

		assert!(b2_abs(polygon1.m_centroid.x - center.x) < abs_tol + rel_tol * b2_abs(center.x));
		assert!(b2_abs(polygon1.m_centroid.y - center.y) < abs_tol + rel_tol * b2_abs(center.y));

		let vertices:[B2vec2;4]=[
		B2vec2::new(center.x - hx, center.y - hy),
		B2vec2::new(center.x + hx, center.y - hy),
		B2vec2::new(center.x - hx, center.y + hy),
		B2vec2::new(center.x + hx, center.y + hy),
		];

		let mut polygon2 = B2polygonShape::default();
		polygon2.set(&vertices);

		assert!(b2_abs(polygon2.m_centroid.x - center.x) < abs_tol + rel_tol * b2_abs(center.x));
		assert!(b2_abs(polygon2.m_centroid.y - center.y) < abs_tol + rel_tol * b2_abs(center.y));

		let mass: f32 = 4.0 * hx * hy;
		let inertia: f32 = (mass / 3.0) * (hx * hx + hy * hy) + mass * b2_dot(center, center);

		let mut mass_data1 = B2massData::default();
		polygon1.compute_mass(&mut mass_data1, 1.0);

		assert!(b2_abs(mass_data1.center.x - center.x) < abs_tol + rel_tol * b2_abs(center.x));
		assert!(b2_abs(mass_data1.center.y - center.y) < abs_tol + rel_tol * b2_abs(center.y));
		assert!(b2_abs(mass_data1.mass - mass) < 20.0 * (abs_tol + rel_tol * mass));
		assert!(b2_abs(mass_data1.i - inertia) < 40.0 * (abs_tol + rel_tol * inertia));

		let mut mass_data2 = B2massData::default();
		polygon2.compute_mass(&mut mass_data2, 1.0);

		assert!(b2_abs(mass_data2.center.x - center.x) < abs_tol + rel_tol * b2_abs(center.x));
		assert!(b2_abs(mass_data2.center.y - center.y) < abs_tol + rel_tol * b2_abs(center.y));
		assert!(b2_abs(mass_data2.mass - mass) < 20.0 * (abs_tol + rel_tol * mass));
		assert!(b2_abs(mass_data2.i - inertia) < 40.0 * (abs_tol + rel_tol * inertia));
	}
}
