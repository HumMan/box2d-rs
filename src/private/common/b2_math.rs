use crate::b2_math::{b2_cross_vec3, b2_dot_vec3, B2Mat33, B2vec2, B2Vec3};

/// solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
pub fn solve33(self_: B2Mat33, b: B2Vec3) -> B2Vec3 {
    let mut det: f32 = b2_dot_vec3(self_.ex, b2_cross_vec3(self_.ey, self_.ez));
    if det != 0.0 {
        det = 1.0 / det;
    }
    let x = B2Vec3 {
        x: det * b2_dot_vec3(b, b2_cross_vec3(self_.ey, self_.ez)),
        y: det * b2_dot_vec3(self_.ex, b2_cross_vec3(b, self_.ez)),
        z: det * b2_dot_vec3(self_.ex, b2_cross_vec3(self_.ey, b)),
    };
    return x;
}
/// solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
pub fn solve22(self_: B2Mat33, b: B2vec2) -> B2vec2 {
    let (a11, a12, a21, a22) = (self_.ex.x, self_.ey.x, self_.ex.y, self_.ey.y);
    let mut det: f32 = a11 * a22 - a12 * a21;
    if det != 0.0 {
        det = 1.0 / det;
    }
    let x = B2vec2 {
        x: det * (a22 * b.x - a12 * b.y),
        y: det * (a11 * b.y - a21 * b.x),
    };
    return x;
}
///
pub fn get_inverse22(self_: B2Mat33, m: &mut B2Mat33) {
    let (a, b, c, d) = (self_.ex.x, self_.ey.x, self_.ex.y, self_.ey.y);
    let mut det: f32 = a * d - b * c;
    if det != 0.0 {
        det = 1.0 / det;
    }

    m.ex.x = det * d;
    m.ey.x = -det * b;
    m.ex.z = 0.0;
    m.ex.y = -det * c;
    m.ey.y = det * a;
    m.ey.z = 0.0;
    m.ez.x = 0.0;
    m.ez.y = 0.0;
    m.ez.z = 0.0;
}
/// Returns the zero matrix if singular.
pub fn get_sym_inverse33(self_: B2Mat33, m: &mut B2Mat33) {
    let mut det = b2_dot_vec3(self_.ex, b2_cross_vec3(self_.ey, self_.ez));
    if det != 0.0 {
        det = 1.0 / det;
    }

    let (a11, a12, a13) = (self_.ex.x, self_.ey.x, self_.ez.x);
    let (a22, a23) = (self_.ey.y, self_.ez.y);
    let a33 = self_.ez.z;

    m.ex.x = det * (a22 * a33 - a23 * a23);
    m.ex.y = det * (a13 * a23 - a12 * a33);
    m.ex.z = det * (a12 * a23 - a13 * a22);

    m.ey.x = m.ex.y;
    m.ey.y = det * (a11 * a33 - a13 * a13);
    m.ey.z = det * (a13 * a12 - a11 * a23);

    m.ez.x = m.ex.z;
    m.ez.y = m.ey.z;
    m.ez.z = det * (a11 * a22 - a12 * a12);
}
