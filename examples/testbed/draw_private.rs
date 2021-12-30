use super::draw::*;
use box2d_rs::b2_collision::*;

use box2d_rs::b2_draw::*;
use box2d_rs::b2_math::*;
use box2d_rs::b2_common::B2_PI;

use glium::backend::Facade;
use glium::Surface;

use imgui::sys;
use imgui::im_str;
use glium::{implement_vertex, uniform};

pub(crate) fn camera_convert_screen_to_world(self_: &Camera, ps: B2vec2) -> B2vec2 {
	let w: f32 = self_.m_width as f32;
	let h: f32 = self_.m_height as f32;
	let u: f32 = ps.x / w;
	let v: f32 = (h - ps.y) / h;

	let ratio: f32 = w / h;
	let mut extents = B2vec2::new(ratio * 25.0, 25.0);
	extents *= self_.m_zoom;

	let lower: B2vec2 = self_.m_center - extents;
	let upper: B2vec2 = self_.m_center + extents;

	let pw = B2vec2 {
		x: (1.0 - u) * lower.x + u * upper.x,
		y: (1.0 - v) * lower.y + v * upper.y,
	};
	return pw;
}

pub(crate) fn convert_world_to_screen(self_: &Camera, pw: B2vec2) -> B2vec2 {
	let w: f32 = self_.m_width as f32;
	let h: f32 = self_.m_height as f32;
	let ratio: f32 = w / h;
	let mut extents = B2vec2::new(ratio * 25.0, 25.0);
	extents *= self_.m_zoom;

	let lower: B2vec2 = self_.m_center - extents;
	let upper: B2vec2 = self_.m_center + extents;

	let u: f32 = (pw.x - lower.x) / (upper.x - lower.x);
	let v: f32 = (pw.y - lower.y) / (upper.y - lower.y);

	let ps = B2vec2 {
		x: u * w,
		y: (1.0 - v) * h,
	};
	return ps;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
pub(crate) fn build_projection_matrix(self_: &Camera, m: &mut [f32; 16], z_bias: f32) {
	let w: f32 = self_.m_width as f32;
	let h: f32 = self_.m_height as f32;
	let ratio: f32 = w / h;
	let mut extents = B2vec2::new(ratio * 25.0, 25.0);
	extents *= self_.m_zoom;

	let lower: B2vec2 = self_.m_center - extents;
	let upper: B2vec2 = self_.m_center + extents;

	m[0] = 2.0 / (upper.x - lower.x);
	m[1] = 0.0;
	m[2] = 0.0;
	m[3] = 0.0;

	m[4] = 0.0;
	m[5] = 2.0 / (upper.y - lower.y);
	m[6] = 0.0;
	m[7] = 0.0;

	m[8] = 0.0;
	m[9] = 0.0;
	m[10] = 1.0;
	m[11] = 0.0;

	m[12] = -(upper.x + lower.x) / (upper.x - lower.x);
	m[13] = -(upper.y + lower.y) / (upper.y - lower.y);
	m[14] = z_bias;
	m[15] = 1.0;
}

#[derive(Debug, Default, Copy, Clone)]
struct PointWithColorSize {
	v_position: [f32; 2],
	v_color: [f32; 4],
	v_size: f32,
}

implement_vertex!(PointWithColorSize, v_position, v_color, v_size);

#[derive(Debug, Default)]
pub(crate) struct GLRenderPoints {
	m_vertices: Vec<PointWithColorSize>,
	//m_vbo: Option<glium::VertexBuffer<PointWithColorSize>>,
	m_program_id: Option<glium::Program>,
}

pub(crate) fn from16to4x4(v: [f32; 16]) -> [[f32; 4]; 4] {
	[
		[v[4 * 0 + 0], v[4 * 0 + 1], v[4 * 0 + 2], v[4 * 0 + 3]],
		[v[4 * 1 + 0], v[4 * 1 + 1], v[4 * 1 + 2], v[4 * 1 + 3]],
		[v[4 * 2 + 0], v[4 * 2 + 1], v[4 * 2 + 2], v[4 * 2 + 3]],
		[v[4 * 3 + 0], v[4 * 3 + 1], v[4 * 3 + 2], v[4 * 3 + 3]],
	]
}

impl GLRenderPoints {
	pub(crate) fn create<F: Facade>(&mut self, display: &F) {
		let source = glium::program::ProgramCreationInput::SourceCode {
			tessellation_control_shader: None,
			tessellation_evaluation_shader: None,
			geometry_shader: None,
			outputs_srgb: false,
			uses_point_size: true,
			vertex_shader: "
				#version 330
				uniform mat4 projectionMatrix;
				layout(location = 0) in vec2 v_position;
				layout(location = 1) in vec4 v_color;
				layout(location = 2) in float v_size;
				out vec4 f_color;
				void main(void)
				{
					f_color = v_color;
					gl_Position = projectionMatrix * vec4(v_position, 0.0, 1.0);
					gl_PointSize = v_size;
				}
			",
			fragment_shader: "
				#version 330
				in vec4 f_color;
				out vec4 color;
				void main(void)
				{
					color = f_color;
				}
			",

			transform_feedback_varyings: None,
		};

		self.m_program_id = match glium::Program::new(display, source) {
			Ok(p) => Some(p),
			Err(glium::program::ProgramCreationError::TransformFeedbackNotSupported) => return,
			Err(e) => panic!("{:?}", e),
		};
		//self.m_vbo = Some(glium::VertexBuffer::empty_dynamic(display, 1).unwrap());
		self.m_vertices.clear();
	}

	pub(crate) fn vertex(&mut self, v: B2vec2, c: B2color, size: f32) {
		self.m_vertices.push(PointWithColorSize {
			v_position: [v.x, v.y],
			v_color: [c.r, c.g, c.b, c.a],
			v_size: size,
		})
	}

	pub(crate) fn flush<F: Facade>(
		&mut self,
		display: &F,
		frame: &mut glium::Frame,
		camera: Camera,
	) {
		//self.m_vbo.as_ref().unwrap().write(&self.m_vertices);
		let vbo = glium::VertexBuffer::new(display, &self.m_vertices).unwrap();
		let mut proj: [f32; 16] = [0.0;16];
		camera.build_projection_matrix(&mut proj, 0.0);

		let uniforms = uniform! {
			projectionMatrix: from16to4x4(proj)
		};

		frame
			.draw(
				&vbo,
				&glium::index::NoIndices(glium::index::PrimitiveType::Points),
				self.m_program_id.as_ref().unwrap(),
				&uniforms,
				&Default::default(),
			)
			.unwrap();

		self.m_vertices.clear();
	}
}

#[derive(Debug, Default, Copy, Clone)]
struct PointWithColor {
	v_position: [f32; 2],
	v_color: [f32; 4],
}

implement_vertex!(PointWithColor, v_position, v_color);

#[derive(Debug, Default)]
pub(crate) struct GLRenderLines {
	m_vertices: Vec<PointWithColor>,
	//m_vbo: Option<glium::VertexBuffer<PointWithColor>>,
	m_program_id: Option<glium::Program>,
}

impl GLRenderLines {
	pub(crate) fn create<F: Facade>(&mut self, display: &F) {
		let vs = r#"
		#version 330
		uniform mat4 projectionMatrix;
		layout(location = 0) in vec2 v_position;
		layout(location = 1) in vec4 v_color;
		out vec4 f_color;
		void main(void)
		{
			f_color = v_color;
			gl_Position = projectionMatrix * vec4(v_position, 0.0, 1.0);
		}
		"#;

		let fs = r#"
        #version 330
		in vec4 f_color;
		out vec4 color;
		void main(void)
		{
			color = f_color;
		}
		"#;

		self.m_program_id = Some(glium::Program::from_source(display, vs, fs, None).unwrap());
		//self.m_vbo = Some(glium::VertexBuffer::empty_dynamic(display, 2).unwrap());
		self.m_vertices.clear();
	}

	pub(crate) fn vertex(&mut self, v: B2vec2, c: B2color) {
		self.m_vertices.push(PointWithColor {
			v_position: [v.x, v.y],
			v_color: [c.r, c.g, c.b, c.a],
		})
	}

	pub(crate) fn flush<F: Facade>(
		&mut self,
		display: &F,
		frame: &mut glium::Frame,
		camera: Camera,
	) {
		//self.m_vbo.as_ref().unwrap().write(&self.m_vertices);
		let vbo = glium::VertexBuffer::new(display, &self.m_vertices).unwrap();
		let mut proj: [f32; 16] = [0.0;16];
		camera.build_projection_matrix(&mut proj, 0.0);

		let uniforms = uniform! {
			projectionMatrix: from16to4x4(proj)
		};

		frame
			.draw(
				&vbo,
				&glium::index::NoIndices(glium::index::PrimitiveType::LinesList),
				self.m_program_id.as_ref().unwrap(),
				&uniforms,
				&Default::default(),
			)
			.unwrap();

		self.m_vertices.clear();
	}
}

#[derive(Debug, Default)]
pub(crate) struct GLRenderTriangles {
	m_vertices: Vec<PointWithColor>,
	//m_vbo: Option<glium::VertexBuffer<PointWithColor>>,
	m_program_id: Option<glium::Program>,
}

impl GLRenderTriangles {
	pub(crate) fn create<F: Facade>(&mut self, display: &F) {
		let vs = r#"
		#version 330
		uniform mat4 projectionMatrix;
		layout(location = 0) in vec2 v_position;
		layout(location = 1) in vec4 v_color;
		out vec4 f_color;
		void main(void)
		{
			f_color = v_color;
			gl_Position = projectionMatrix * vec4(v_position, 0.0, 1.0);
		}
		"#;

		let fs = r#"
        #version 330
		in vec4 f_color;
		out vec4 color;
		void main(void)
		{
			color = f_color;
		}
		"#;

		self.m_program_id = Some(glium::Program::from_source(display, vs, fs, None).unwrap());
		//self.m_vbo = Some();
		self.m_vertices.clear();
	}

	pub(crate) fn vertex(&mut self, v: B2vec2, c: B2color) {
		self.m_vertices.push(PointWithColor {
			v_position: [v.x, v.y],
			v_color: [c.r, c.g, c.b, c.a],
		})
	}

	pub(crate) fn flush<F: Facade>(
		&mut self,
		display: &F,
		frame: &mut glium::Frame,
		camera: Camera,
	) {
		//self.m_vbo.as_ref().unwrap().write(&self.m_vertices);
		let vbo = glium::VertexBuffer::new(display, &self.m_vertices).unwrap();
		let mut proj: [f32; 16] = [0.0;16];
		camera.build_projection_matrix(&mut proj, 0.0);

		let uniforms = uniform! {
			projectionMatrix: from16to4x4(proj)
		};

		let params = glium::DrawParameters {
			blend: glium::Blend::alpha_blending(),
			..Default::default()
		};

		frame
			.draw(
				&vbo,
				&glium::index::NoIndices(glium::index::PrimitiveType::TrianglesList),
				self.m_program_id.as_ref().unwrap(),
				&uniforms,
				&params,
			)
			.unwrap();

		self.m_vertices.clear();
	}
}

pub(crate) fn create<F: Facade>(self_: &mut TestBedDebugDraw, display: &F) {
	self_.m_points.create(display);
	self_.m_lines.create(display);
	self_.m_triangles.create(display);
}

// pub(crate) fn destroy(self_: &mut TestBedDebugDraw) {
// 	self_.m_points = GLRenderPoints::default();
// 	self_.m_lines = GLRenderLines::default();
// 	self_.m_triangles = GLRenderTriangles::default();
// }

pub(crate) fn draw_polygon(self_: &mut TestBedDebugDraw, vertices: &[B2vec2], color: B2color) {
	let vertex_count = vertices.len();
	let mut p1: B2vec2 = vertices[vertex_count - 1];
	for i in 0..vertex_count {
		let p2: B2vec2 = vertices[i];
		self_.m_lines.vertex(p1, color);
		self_.m_lines.vertex(p2, color);
		p1 = p2;
	}
}

pub(crate) fn draw_solid_polygon(
	self_: &mut TestBedDebugDraw,
	vertices: &[B2vec2],
	color: B2color,
) {
	let fill_color = B2color::new_with_alpha(0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5);
	let vertex_count = vertices.len();
	for i in 1..vertex_count - 1 {
		self_.m_triangles.vertex(vertices[0], fill_color);
		self_.m_triangles.vertex(vertices[i], fill_color);
		self_.m_triangles.vertex(vertices[i + 1], fill_color);
	}

	let mut p1: B2vec2 = vertices[vertex_count - 1];
	for i in 0..vertex_count {
		let p2: B2vec2 = vertices[i];
		self_.m_lines.vertex(p1, color);
		self_.m_lines.vertex(p2, color);
		p1 = p2;
	}
}

pub(crate) fn draw_circle(
	self_: &mut TestBedDebugDraw,
	center: B2vec2,
	radius: f32,
	color: B2color,
) {
	const K_SEGMENTS: i32 = 16;
	const K_INCREMENT: f32 = 2.0 * B2_PI / (K_SEGMENTS as f32);
	let sin_inc: f32 = f32::sin(K_INCREMENT);
	let cos_inc: f32 = f32::cos(K_INCREMENT);
	let mut r1 = B2vec2::new(1.0, 0.0);
	let mut v1: B2vec2 = center + radius * r1;
	for _i in 0..K_SEGMENTS {
		// Perform rotation to avoid additional trigonometry.
		let r2 = B2vec2 {
			x: cos_inc * r1.x - sin_inc * r1.y,
			y: sin_inc * r1.x + cos_inc * r1.y,
		};
		let v2: B2vec2 = center + radius * r2;
		self_.m_lines.vertex(v1, color);
		self_.m_lines.vertex(v2, color);
		r1 = r2;
		v1 = v2;
	}
}

pub(crate) fn draw_solid_circle(
	self_: &mut TestBedDebugDraw,
	center: B2vec2,
	radius: f32,
	axis: B2vec2,
	color: B2color,
) {
	const K_SEGMENTS: i32 = 16;
	const K_INCREMENT: f32 = 2.0 * B2_PI / (K_SEGMENTS as f32);
	let sin_inc: f32 = f32::sin(K_INCREMENT);
	let cos_inc: f32 = f32::cos(K_INCREMENT);
	let v0: B2vec2 = center;
	let mut r1 = B2vec2::new(cos_inc, sin_inc);
	let mut v1: B2vec2 = center + radius * r1;
	let fill_color = B2color::new_with_alpha(0.5 * color.r, 0.5 * color.g, 0.5 * color.b, 0.5);
	for _i in 0..K_SEGMENTS {
		// Perform rotation to avoid additional trigonometry.
		let r2 = B2vec2 {
			x: cos_inc * r1.x - sin_inc * r1.y,
			y: sin_inc * r1.x + cos_inc * r1.y,
		};
		let v2: B2vec2 = center + radius * r2;
		self_.m_triangles.vertex(v0, fill_color);
		self_.m_triangles.vertex(v1, fill_color);
		self_.m_triangles.vertex(v2, fill_color);
		r1 = r2;
		v1 = v2;
	}

	r1.set(1.0, 0.0);
	v1 = center + radius * r1;
	for _i in 0..K_SEGMENTS {
		let r2 = B2vec2 {
			x: cos_inc * r1.x - sin_inc * r1.y,
			y: sin_inc * r1.x + cos_inc * r1.y,
		};

		let v2: B2vec2 = center + radius * r2;
		self_.m_lines.vertex(v1, color);
		self_.m_lines.vertex(v2, color);
		r1 = r2;
		v1 = v2;
	}

	// Draw a line fixed in the circle to animate rotation.
	let p: B2vec2 = center + radius * axis;
	self_.m_lines.vertex(center, color);
	self_.m_lines.vertex(p, color);
}

pub(crate) fn draw_segment(self_: &mut TestBedDebugDraw, p1: B2vec2, p2: B2vec2, color: B2color) {
	self_.m_lines.vertex(p1, color);
	self_.m_lines.vertex(p2, color);
}

pub(crate) fn draw_transform(self_: &mut TestBedDebugDraw, xf: B2Transform) {
	const K_AXIS_SCALE: f32 = 0.4;
	let red = B2color::new(1.0, 0.0, 0.0);
	let green = B2color::new(0.0, 1.0, 0.0);
	let p1: B2vec2 = xf.p;
	let mut p2;

	self_.m_lines.vertex(p1, red);
	p2 = p1 + K_AXIS_SCALE * xf.q.get_xaxis();
	self_.m_lines.vertex(p2, red);

	self_.m_lines.vertex(p1, green);
	p2 = p1 + K_AXIS_SCALE * xf.q.get_yaxis();
	self_.m_lines.vertex(p2, green);
}

pub(crate) fn draw_point(self_: &mut TestBedDebugDraw, p: B2vec2, size: f32, color: B2color) {
	self_.m_points.vertex(p, color, size);
}

pub(crate) fn draw_string(self_: &TestBedDebugDraw, ui: &imgui::Ui<'_>, p: B2vec2, text: &str) {
	if self_.m_show_ui == false {
		return;
	}
	imgui::Window::new(im_str!("Overlay"))
		.flags(imgui::WindowFlags::NO_DECORATION | imgui::WindowFlags::NO_BACKGROUND | imgui::WindowFlags::NO_INPUTS)
		.position([0.0, 0.0], imgui::Condition::Always)
		.size([2000.0, 2000.0], imgui::Condition::Always)
		.build(ui, || {
			unsafe {
				sys::igSetCursorPos(sys::ImVec2{x:p.x,y:p.y});
			}
			ui.text_colored(
				[230.0 / 255.0, 153.0 / 255.0, 153.0 / 255.0, 255.0 / 255.0],
				text,
			);
		});
}

pub(crate) fn draw_string_world(self_: &TestBedDebugDraw, ui: &imgui::Ui<'_>, camera: Camera, pw: B2vec2, text: &str) {
	if self_.m_show_ui == false {
		return;
	}

	let ps: B2vec2 = camera.convert_world_to_screen(pw);
	draw_string(self_, ui, ps, text);
}

pub(crate) fn draw_aabb(self_: &mut TestBedDebugDraw, aabb: B2AABB, c: B2color) {
	let p1: B2vec2 = aabb.lower_bound;
	let p2: B2vec2 = B2vec2::new(aabb.upper_bound.x, aabb.lower_bound.y);
	let p3: B2vec2 = aabb.upper_bound;
	let p4: B2vec2 = B2vec2::new(aabb.lower_bound.x, aabb.upper_bound.y);
	self_.m_lines.vertex(p1, c);
	self_.m_lines.vertex(p2, c);

	self_.m_lines.vertex(p2, c);
	self_.m_lines.vertex(p3, c);

	self_.m_lines.vertex(p3, c);
	self_.m_lines.vertex(p4, c);

	self_.m_lines.vertex(p4, c);
	self_.m_lines.vertex(p1, c);
}

pub(crate) fn flush<F: Facade>(
	self_: &mut TestBedDebugDraw,
	display: &F,
	target: &mut glium::Frame,
	camera: Camera,
) {
	self_.m_triangles.flush(display, target, camera);
	self_.m_lines.flush(display, target, camera);
	self_.m_points.flush(display, target, camera);
}
