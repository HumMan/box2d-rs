use super::super::test::*;

use glium::backend::Facade;

use super::add_pair::AddPair;
use super::apply_force::ApplyForce;
use super::body_types::BodyTypes;
use super::box_stack::BoxStack;
use super::breakable::Breakable;
use super::bridge::Bridge;
use super::bullet_test::BulletTest;
use super::cantilever::Cantilever;
use super::car::Car;
use super::chain::Chain;
use super::chain_problem::ChainProblem;
use super::character_collision::CharacterCollision;
use super::circle_stack::CircleStack;
use super::collision_filtering::CollisionFiltering;
use super::collision_processing::CollisionProcessing;
use super::compound_shapes::CompoundShapes;
use super::compound_shapes_old::CompoundShapesOld;
use super::confined::Confined;
use super::continuous_test::ContinuousTest;
use super::convex_hull::ConvexHull;
use super::conveyor_belt::ConveyorBelt;
use super::distance_joint::DistanceJoint;
use super::distance_test::DistanceTest;
use super::dominos::Dominos;
use super::dump_loader::DumpLoader;
use super::dynamic_tree::DynamicTree;
use super::edge_shapes::EdgeShapes;
use super::edge_test::EdgeTest;
use super::friction::Friction;
use super::gear_joint::GearJoint;
use super::heavy1::Heavy1;
use super::heavy2::Heavy2;
use super::mobile_balanced::MobileBalanced;
use super::mobile_unbalanced::MobileUnbalanced;
use super::motor_joint::MotorJoint;
use super::pinball::Pinball;
use super::platformer::Platformer;
use super::polygon_collision::PolygonCollision;
use super::polygon_shapes::PolygonShapes;
use super::prismatic_joint::PrismaticJoint;
use super::pulley_joint::PulleyJoint;
use super::pyramid::Pyramid;
use super::ray_cast::RayCast;
use super::restitution::Restitution;
use super::revolute_joint::RevoluteJoint;
use super::rope::Rope;
use super::rope_joint::RopeJoint;
use super::sensor::Sensors;
use super::shape_cast::ShapeCast;
use super::shape_editing::ShapeEditing;
use super::skier::Skier;
use super::slider_crank_1::SliderCrank1;
use super::slider_crank_2::SliderCrank2;
use super::theo_jansen::TheoJansen;
use super::tiles::Tiles;
use super::time_of_impact::TimeOfImpact;
use super::tumbler::Tumbler;
use super::wheel_joint::WheelJoint;

macro_rules! add_test {
    ($vec_name:ident, $cat:expr, $name:expr, $constr:ident) => {
        $vec_name.push(TestEntry {
            index: 0,
            category: $cat,
            name: $name,
            create_fcn: $constr::new,
        });
    };
}

pub(crate) fn get_tests_list<'a, F: Facade>() -> Vec<TestEntry<'a, UserDataTypes, F>> {
    let mut r = Vec::<TestEntry<UserDataTypes, F>>::new();

    add_test!(r, "Benchmark", "Add Pair", AddPair);
    add_test!(r, "Forces", "Apply Force", ApplyForce);
    add_test!(r, "Examples", "Body Types", BodyTypes);
    add_test!(r, "Stacking", "Boxes", BoxStack);
    add_test!(r, "Examples", "Breakable", Breakable);
    add_test!(r, "Joints", "Bridge", Bridge);
    add_test!(r, "Continuous", "Bullet Test", BulletTest);
    add_test!(r, "Joints", "Cantilever", Cantilever);
    add_test!(r, "Examples", "Car", Car);
    add_test!(r, "Bugs", "Chain Problem", ChainProblem);
    add_test!(r, "Joints", "Chain", Chain);
    add_test!(r, "Examples", "Character Collision", CharacterCollision);
    add_test!(r, "Stacking", "Circles", CircleStack);
    add_test!(r, "Examples", "Collision Filtering", CollisionFiltering);
    add_test!(r, "Examples", "Collision Processing", CollisionProcessing);
    add_test!(r, "Examples", "Compound Shapes", CompoundShapes);
    add_test!(r, "Examples", "Compound Shapes (<2.4.0)", CompoundShapesOld);
    add_test!(r, "Solver", "Confined", Confined);
    add_test!(r, "Continuous", "Continuous Test", ContinuousTest);
    add_test!(r, "Geometry", "Convex Hull", ConvexHull);
    add_test!(r, "Examples", "Conveyor Belt", ConveyorBelt);
    add_test!(r, "Joints", "Distance Joint", DistanceJoint);
    add_test!(r, "Geometry", "Distance Test", DistanceTest);
    add_test!(r, "Examples", "Dominos", Dominos);
    add_test!(r, "Bugs", "Dump Loader", DumpLoader);
    add_test!(r, "Collision", "Dynamic Tree", DynamicTree);
    add_test!(r, "Geometry", "Edge Shapes", EdgeShapes);
    add_test!(r, "Geometry", "Edge Test", EdgeTest);
    add_test!(r, "Forces", "Friction", Friction);
    add_test!(r, "Joints", "Gear", GearJoint);
    add_test!(r, "Solver", "Heavy 1", Heavy1);
    add_test!(r, "Solver", "Heavy 2", Heavy2);
    add_test!(r, "Solver", "Mobile Balanced", MobileBalanced);
    add_test!(r, "Solver", "Mobile Unbalanced", MobileUnbalanced);
    add_test!(r, "Joints", "Motor Joint", MotorJoint);
    add_test!(r, "Examples", "Pinball", Pinball);
    add_test!(r, "Examples", "Platformer", Platformer);
    add_test!(r, "Geometry", "Polygon Collision", PolygonCollision);
    add_test!(r, "Geometry", "Polygon Shapes", PolygonShapes);
    add_test!(r, "Joints", "Prismatic", PrismaticJoint);
    add_test!(r, "Joints", "Pulley", PulleyJoint);
    add_test!(r, "Stacking", "Pyramid", Pyramid);
    add_test!(r, "Collision", "Ray Cast", RayCast);
    add_test!(r, "Forces", "Restitution", Restitution);
    add_test!(r, "Joints", "Revolute", RevoluteJoint);
    add_test!(r, "Joints", "Rope", RopeJoint);
    add_test!(r, "Rope", "Bending", Rope);
    add_test!(r, "Collision", "Sensors", Sensors);
    add_test!(r, "Collision", "Shape Cast", ShapeCast);
    add_test!(r, "Examples", "Shape Editing", ShapeEditing);
    add_test!(r, "Bugs", "Skier", Skier);
    add_test!(r, "Examples", "Slider Crank 1", SliderCrank1);
    add_test!(r, "Examples", "Slider Crank 2", SliderCrank2);
    add_test!(r, "Examples", "Theo Jansen", TheoJansen);
    add_test!(r, "Benchmark", "Tiles", Tiles);
    add_test!(r, "Collision", "Time of Impact", TimeOfImpact);
    add_test!(r, "Benchmark", "Tumbler", Tumbler);
    add_test!(r, "Joints", "Wheel", WheelJoint);

    r.sort_by(|a, b| a.category.cmp(b.category).then(a.name.cmp(b.name)));

    for (index, v) in r.iter_mut().enumerate() {
        v.index = index;
    }

    return r;
}
