use box2d_rs::b2_collision::*;
use box2d_rs::b2_draw::*;
use box2d_rs::b2_math::*;

use super::draw_private as private;
use glium::backend::Facade;

use std::cell::RefCell;
use std::rc::Rc;

impl Default for Camera {
    fn default() -> Self {
        return Self {
            m_center: B2vec2::new(0.0, 20.0),
            m_zoom: 1.0,
            m_width: 1280,
            m_height: 800,
        };
    }
}

impl Camera {
    pub fn convert_screen_to_world(&self, screen_point: B2vec2) -> B2vec2 {
        return private::camera_convert_screen_to_world(self, screen_point);
    }
    pub fn convert_world_to_screen(&self, world_point: B2vec2) -> B2vec2 {
        return private::convert_world_to_screen(self, world_point);
    }
    pub fn build_projection_matrix(&self, m: &mut [f32; 16], z_bias: f32) {
        private::build_projection_matrix(self, m, z_bias);
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Camera {
    pub(crate) m_center: B2vec2,
    pub(crate) m_zoom: f32,
    pub(crate) m_width: i32,
    pub(crate) m_height: i32,
}

pub(crate) type TestBedDebugDrawPtr = Rc<RefCell<TestBedDebugDraw>>;

#[derive(Debug, Default)]
pub(crate) struct TestBedDebugDraw {
    pub base: B2draw,

    pub m_show_ui: bool,
    pub m_points: private::GLRenderPoints,
    pub m_lines: private::GLRenderLines,
    pub m_triangles: private::GLRenderTriangles,
}

impl TestBedDebugDraw {
    pub fn new() -> Self {
        Self {
            m_show_ui: true,
            ..Default::default()
        }
    }

    pub fn create<F: Facade>(&mut self, display: &F) {
        private::create(self, display);
    }

    // pub fn destroy(&mut self) {
    //     private::destroy(self);
    // }

    pub fn draw_string(&self, ui: &imgui::Ui, p: B2vec2, text: &str) {
        private::draw_string(self, ui, p, text);
    }

    pub fn draw_string_world(&self, ui: &imgui::Ui, camera: Camera, pw: B2vec2, text: &str) {
        private::draw_string_world(self, ui, camera, pw, text);
    }

    pub fn draw_aabb(&mut self, aabb: B2AABB, color: B2color) {
        private::draw_aabb(self, aabb, color);
    }

    pub fn flush<F: Facade>(&mut self, display: &F, target: &mut glium::Frame, camera: Camera) {
        private::flush(self, display, target, camera);
    }
}

// This class implements debug drawing callbacks that are invoked
// inside b2World::Step.
impl B2drawTrait for TestBedDebugDraw {
    fn get_base(&self) -> &B2draw {
        return &self.base;
    }
    fn get_base_mut(&mut self) -> &mut B2draw {
        return &mut self.base;
    }
    /// draw a closed polygon provided in CCW order.
    fn draw_polygon(&mut self, vertices: &[B2vec2], color: B2color) {
        private::draw_polygon(self, vertices, color);
    }

    /// draw a solid closed polygon provided in CCW order.
    fn draw_solid_polygon(&mut self, vertices: &[B2vec2], color: B2color) {
        private::draw_solid_polygon(self, vertices, color);
    }

    /// draw a circle.
    fn draw_circle(&mut self, center: B2vec2, radius: f32, color: B2color) {
        private::draw_circle(self, center, radius, color);
    }

    /// draw a solid circle.
    fn draw_solid_circle(&mut self, center: B2vec2, radius: f32, axis: B2vec2, color: B2color) {
        private::draw_solid_circle(self, center, radius, axis, color);
    }

    /// draw a line segment.
    fn draw_segment(&mut self, p1: B2vec2, p2: B2vec2, color: B2color) {
        private::draw_segment(self, p1, p2, color);
    }

    /// draw a transform. Choose your own length scale.
    /// * `xf` - a transform.
    fn draw_transform(&mut self, xf: B2Transform) {
        private::draw_transform(self, xf);
    }

    /// draw a point.
    fn draw_point(&mut self, p: B2vec2, size: f32, color: B2color) {
        private::draw_point(self, p, size, color);
    }
}
