use crate::b2_math::{B2Transform, B2vec2};

use std::cell::RefCell;
use std::rc::Rc;
use bitflags::bitflags;

/// Color for debug drawing. Each value has the range [0,1].
#[derive(Copy, Clone, Debug)]
pub struct B2color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}
impl B2color {
    pub fn new(r_in: f32, g_in: f32, b_in: f32) -> B2color {
        return B2color {
            r: r_in,
            g: g_in,
            b: b_in,
            a: 1.0,
        };
    }

    pub fn new_with_alpha(r_in: f32, g_in: f32, b_in: f32, a_in: f32) -> B2color {
        return B2color {
            r: r_in,
            g: g_in,
            b: b_in,
            a: a_in,
        };
    }

    pub fn set(&mut self, r_in: f32, g_in: f32, b_in: f32) {
        self.r = r_in;
        self.g = g_in;
        self.b = b_in;
    }

    pub fn set_with_alpha(&mut self, r_in: f32, g_in: f32, b_in: f32, a_in: f32) {
        self.r = r_in;
        self.g = g_in;
        self.b = b_in;
        self.a = a_in;
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct B2draw {
    m_draw_flags: B2drawShapeFlags,
}

bitflags! {
    #[derive(Debug, Clone, Copy, Default)]
    pub struct B2drawShapeFlags: u16 {
    /// draw shapes
    const SHAPE_BIT = 0x0001;
    /// draw joint connections
    const JOINT_BIT = 0x0002;
    /// draw axis aligned bounding boxes
    const AABB_BIT = 0x0004;
    /// draw broad-phase pairs
    const PAIR_BIT = 0x0008;
    /// draw center of mass frame
    const CENTER_OF_MASS_BIT = 0x0010;
}
}

impl B2draw {
    /// Set the drawing flags.
    pub fn set_flags(&mut self, flags: B2drawShapeFlags) {
        self.m_draw_flags = flags;
    }

    /// Get the drawing flags.
    pub fn get_flags(self) -> B2drawShapeFlags {
        return self.m_draw_flags;
    }

    /// Append flags to the current flags.
    pub fn append_flags(&mut self, flags: B2drawShapeFlags) {
        self.m_draw_flags.insert(flags);
    }

    /// clear flags from the current flags.
    pub fn clear_flags(&mut self, flags: B2drawShapeFlags) {
        self.m_draw_flags.remove(flags);
    }
}

pub type B2drawTraitPtr = Rc<RefCell<dyn B2drawTrait>>;

pub trait B2drawTrait {
    fn get_base(&self) -> &B2draw;
    fn get_base_mut(&mut self) -> &mut B2draw;
    /// draw a closed polygon provided in CCW order.
    fn draw_polygon(&mut self, vertices: &[B2vec2], color: B2color);

    /// draw a solid closed polygon provided in CCW order.
    fn draw_solid_polygon(&mut self, vertices: &[B2vec2], color: B2color);

    /// draw a circle.
    fn draw_circle(&mut self, center: B2vec2, radius: f32, color: B2color);

    /// draw a solid circle.
    fn draw_solid_circle(&mut self, center: B2vec2, radius: f32, axis: B2vec2, color: B2color);

    /// draw a line segment.
    fn draw_segment(&mut self, p1: B2vec2, p2: B2vec2, color: B2color);

    /// draw a transform. Choose your own length scale.
    /// * `xf` - a transform.
    fn draw_transform(&mut self, xf: B2Transform);

    /// draw a point.
    fn draw_point(&mut self, p: B2vec2, size: f32, color: B2color);
}
