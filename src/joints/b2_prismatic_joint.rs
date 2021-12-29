use crate::b2_body::*;
use crate::b2_draw::*;
use crate::b2_joint::*;
use crate::b2_math::*;
use crate::b2_common::*;
use crate::b2rs_common::UserDataType;
use crate::b2_time_step::*;
use crate::private::dynamics::joints::b2_prismatic_joint as private;

impl<D: UserDataType> Default for B2prismaticJointDef<D> {
    fn default() -> Self {
        return Self {
            base: B2jointDef {
                jtype: B2jointType::EPrismaticJoint,
                ..Default::default()
            },
            local_anchor_a: B2vec2::zero(),
            local_anchor_b: B2vec2::zero(),
            local_axis_a: B2vec2::new(1.0, 0.0),
            reference_angle: 0.0,
            enable_limit: false,
            lower_translation: 0.0,
            upper_translation: 0.0,
            enable_motor: false,
            max_motor_force: 0.0,
            motor_speed: 0.0,
        };
    }
}

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
#[derive(Clone)]
pub struct B2prismaticJointDef<D: UserDataType> {
    pub base: B2jointDef<D>,

    /// The local anchor point relative to body_a's origin.
    pub local_anchor_a: B2vec2,

    /// The local anchor point relative to body_b's origin.
    pub local_anchor_b: B2vec2,

    /// The local translation unit axis in body_a.
    pub local_axis_a: B2vec2,

    /// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
    pub reference_angle: f32,

    /// Enable/disable the joint limit.
    pub enable_limit: bool,

    /// The lower translation limit, usually in meters.
    pub lower_translation: f32,

    /// The upper translation limit, usually in meters.
    pub upper_translation: f32,

    /// Enable/disable the joint motor.
    pub enable_motor: bool,

    /// The maximum motor torque, usually in n-m.
    pub max_motor_force: f32,

    /// The desired motor speed in radians per second.
    pub motor_speed: f32,
}

impl<D: UserDataType> B2prismaticJointDef<D> {
    /// initialize the bodies, anchors, axis, and reference angle using the world
    /// anchor and unit world axis.
    pub fn initialize(
        &mut self,
        body_a: BodyPtr<D>,
        body_b: BodyPtr<D>,
        anchor: B2vec2,
        axis: B2vec2,
    ) {
        self.base.body_a = Some(body_a.clone());
        self.base.body_b = Some(body_b.clone());
        self.local_anchor_a = body_a.borrow().get_local_point(anchor);
        self.local_anchor_b = body_b.borrow().get_local_point(anchor);
        self.local_axis_a = body_a.borrow().get_local_vector(axis);
        self.reference_angle = body_b.borrow().get_angle() - body_a.borrow().get_angle();
    }
}

impl<D: UserDataType> ToDerivedJoint<D> for B2prismaticJoint<D> {
    fn as_derived(&self) -> JointAsDerived<D> {
        return JointAsDerived::EPrismaticJoint(self);
    }
    fn as_derived_mut(&mut self) -> JointAsDerivedMut<D> {
        return JointAsDerivedMut::EPrismaticJoint(self);
    }
}

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in body_a. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
pub struct B2prismaticJoint<D: UserDataType> {
    pub(crate) base: B2joint<D>,

    pub(crate) m_local_anchor_a: B2vec2,
    pub(crate) m_local_anchor_b: B2vec2,
    pub(crate) m_local_xaxis_a: B2vec2,
    pub(crate) m_local_yaxis_a: B2vec2,
    pub(crate) m_reference_angle: f32,
    pub(crate) m_impulse: B2vec2,
    pub(crate) m_motor_impulse: f32,
    pub(crate) m_lower_impulse: f32,
    pub(crate) m_upper_impulse: f32,
    pub(crate) m_lower_translation: f32,
    pub(crate) m_upper_translation: f32,
    pub(crate) m_max_motor_force: f32,
    pub(crate) m_motor_speed: f32,
    pub(crate) m_enable_limit: bool,
    pub(crate) m_enable_motor: bool,

    // Solver temp
    pub(crate) m_index_a: i32,
    pub(crate) m_index_b: i32,
    pub(crate) m_local_center_a: B2vec2,
    pub(crate) m_local_center_b: B2vec2,
    pub(crate) m_inv_mass_a: f32,
    pub(crate) m_inv_mass_b: f32,
    pub(crate) m_inv_ia: f32,
    pub(crate) m_inv_ib: f32,
    pub(crate) m_axis: B2vec2,
    pub(crate) m_perp: B2vec2,
    pub(crate) m_s1: f32,
    pub(crate) m_s2: f32,
    pub(crate) m_a1: f32,
    pub(crate) m_a2: f32,
    pub(crate) m_k: B2Mat22,
    pub(crate) m_translation: f32,
    pub(crate) m_axial_mass: f32,
}

impl<D: UserDataType> B2prismaticJoint<D> {
    /// The local anchor point relative to body_a's origin.
    pub fn get_local_anchor_a(&self) -> B2vec2 {
        return self.m_local_anchor_a;
    }

    /// The local anchor point relative to body_b's origin.
    pub fn get_local_anchor_b(&self) -> B2vec2 {
        return self.m_local_anchor_b;
    }

    /// The local joint axis relative to body_a.
    pub fn get_local_axis_a(&self) -> B2vec2 {
        return self.m_local_xaxis_a;
    }

    /// Get the reference angle.
    pub fn get_reference_angle(&self) -> f32 {
        return self.m_reference_angle;
    }

    /// Get the current joint translation, usually in meters.
    pub fn get_joint_translation(&self) -> f32 {
        return private::get_joint_translation(self);
    }

    /// Get the current joint translation speed, usually in meters per second.
    pub fn get_joint_speed(&self) -> f32 {
        return private::get_joint_speed(self);
    }

    /// Is the joint limit enabled?
    pub  fn is_limit_enabled(&self) -> bool {
        return self.m_enable_limit;
    }

    /// Enable/disable the joint limit.
    pub fn enable_limit(&mut self, flag: bool) {
        if flag != self.m_enable_limit {
            self.base.m_body_a.borrow_mut().set_awake(true);
            self.base.m_body_b.borrow_mut().set_awake(true);
            self.m_enable_limit = flag;
            self.m_lower_impulse = 0.0;
            self.m_upper_impulse = 0.0;
        }
    }

    /// Get the lower joint limit, usually in meters.
    pub fn get_lower_limit(&self) -> f32 {
        return self.m_lower_translation;
    }

    /// Get the upper joint limit, usually in meters.
    pub fn get_upper_limit(&self) -> f32 {
        return self.m_upper_translation;
    }

    /// Set the joint limits, usually in meters.
    pub fn set_limits(&mut self, lower: f32, upper: f32) {
        b2_assert(lower <= upper);
        if lower != self.m_lower_translation || upper != self.m_upper_translation {
            self.base.m_body_a.borrow_mut().set_awake(true);
            self.base.m_body_b.borrow_mut().set_awake(true);
            self.m_lower_translation = lower;
            self.m_upper_translation = upper;
            self.m_lower_impulse = 0.0;
            self.m_upper_impulse = 0.0;
        }
    }

    /// Is the joint motor enabled?
    pub fn is_motor_enabled(&self) -> bool {
        return self.m_enable_motor;
    }

    /// Enable/disable the joint motor.
    pub fn enable_motor(&mut self, flag: bool) {
        if flag != self.m_enable_motor {
            self.base.m_body_a.borrow_mut().set_awake(true);
            self.base.m_body_b.borrow_mut().set_awake(true);
            self.m_enable_motor = flag;
        }
    }

    /// Set the motor speed, usually in meters per second.
    pub fn set_motor_speed(&mut self, speed: f32) {
        if speed != self.m_motor_speed {
            self.base.m_body_a.borrow_mut().set_awake(true);
            self.base.m_body_b.borrow_mut().set_awake(true);
            self.m_motor_speed = speed;
        }
    }

    /// Get the motor speed, usually in meters per second.
    pub fn get_motor_speed(&self) -> f32 {
        return self.m_motor_speed;
    }

    /// Set the maximum motor force, usually in n.
    pub fn set_max_motor_force(&mut self, force: f32) {
        if force != self.m_max_motor_force {
            self.base.m_body_a.borrow_mut().set_awake(true);
            self.base.m_body_b.borrow_mut().set_awake(true);
            self.m_max_motor_force = force;
        }
    }

    pub fn get_max_motor_force(&self) -> f32 {
        return self.m_max_motor_force;
    }

    /// Get the current motor force given the inverse time step, usually in n.
    pub  fn get_motor_force(&self, inv_dt: f32) -> f32 {
        return inv_dt * self.m_motor_impulse;
    }

    pub(crate) fn new(def: &B2prismaticJointDef<D>) -> Self {
        return private::new(def);
    }
}
impl<D: UserDataType> B2jointTraitDyn<D> for B2prismaticJoint<D> {
    fn get_base(&self) -> &B2joint<D> {
        return &self.base;
    }
    fn get_base_mut(&mut self) -> &mut B2joint<D> {
        return &mut self.base;
    }
    fn get_anchor_a(&self) -> B2vec2 {
        return self
            .base
            .m_body_a
            .borrow()
            .get_world_point(self.m_local_anchor_a);
    }
    fn get_anchor_b(&self) -> B2vec2 {
        return self
            .base
            .m_body_b
            .borrow()
            .get_world_point(self.m_local_anchor_b);
    }

    /// Get the reaction force given the inverse time step.
    /// Unit is n.
    fn get_reaction_force(&self, inv_dt: f32) -> B2vec2 {
        return inv_dt
            * (self.m_impulse.x * self.m_perp
                + (self.m_motor_impulse + self.m_lower_impulse - self.m_upper_impulse) * self.m_axis);
    }

    fn get_reaction_torque(&self, inv_dt: f32) -> f32 {
        return inv_dt * self.m_impulse.y;
    }
    fn init_velocity_constraints(
        &mut self,
        data: &mut B2solverData,
        positions: &mut [B2position],
        velocities: &mut [B2velocity],
    ) {
        private::init_velocity_constraints(self, data, positions, velocities);
    }
    fn solve_velocity_constraints(
        &mut self,
        data: &mut B2solverData,
        velocities: &mut [B2velocity],
    ) {
        private::solve_velocity_constraints(self, data, velocities);
    }
    fn solve_position_constraints(
        &mut self,
        data: &mut B2solverData,
        positions: &mut [B2position],
    ) -> bool {
        return private::solve_position_constraints(self, data, positions);
    }

    ///
    fn draw(&self, draw: &mut dyn B2drawTrait) {
        private::draw(self, draw);
    }
}
