use super::b2_chain_shape::*;
use super::b2_circle_shape::*;
use super::b2_edge_shape::*;
use super::b2_polygon_shape::*;

pub trait ToDerivedShape {

    fn as_derived(&self) -> ShapeAsDerived;

    fn as_circle(&self) -> Option<&B2circleShape>;
    fn as_edge(&self) -> Option<&B2edgeShape>;
    fn as_polygon(&self) -> Option<&B2polygonShape>;
    fn as_chain(&self) -> Option<&B2chainShape>;
}

pub enum ShapeAsDerived<'a>
{
    AsCircle(&'a B2circleShape),
    AsEdge(&'a B2edgeShape),
    AsPolygon(&'a B2polygonShape),
    AsChain(&'a B2chainShape),
}

impl ToDerivedShape for B2circleShape {

    fn as_derived(&self) -> ShapeAsDerived{
        return ShapeAsDerived::AsCircle(&self);
    }

    fn as_circle(&self) -> Option<&B2circleShape> {
        Some(self)
    }

    fn as_edge(&self) -> Option<&B2edgeShape> {
        None
    }

    fn as_polygon(&self) -> Option<&B2polygonShape> {
        None
    }

    fn as_chain(&self) -> Option<&B2chainShape> {
        None
    }
}

impl ToDerivedShape for B2edgeShape {
    fn as_derived(&self) -> ShapeAsDerived{
        return ShapeAsDerived::AsEdge(&self);
    }

    fn as_circle(&self) -> Option<&B2circleShape> {
        None
    }

    fn as_edge(&self) -> Option<&B2edgeShape> {
        Some(self)
    }

    fn as_polygon(&self) -> Option<&B2polygonShape> {
        None
    }

    fn as_chain(&self) -> Option<&B2chainShape> {
        None
    }
}

impl ToDerivedShape for B2polygonShape {
    fn as_derived(&self) -> ShapeAsDerived{
        return ShapeAsDerived::AsPolygon(&self);
    }
    fn as_circle(&self) -> Option<&B2circleShape> {
        None
    }

    fn as_edge(&self) -> Option<&B2edgeShape> {
        None
    }

    fn as_polygon(&self) -> Option<&B2polygonShape> {
        Some(self)
    }

    fn as_chain(&self) -> Option<&B2chainShape> {
        None
    }
}

impl ToDerivedShape for B2chainShape {
    fn as_derived(&self) -> ShapeAsDerived{
        return ShapeAsDerived::AsChain(&self);
    }
    fn as_circle(&self) -> Option<&B2circleShape> {
        None
    }

    fn as_edge(&self) -> Option<&B2edgeShape> {
        None
    }

    fn as_polygon(&self) -> Option<&B2polygonShape> {
        None
    }

    fn as_chain(&self) -> Option<&B2chainShape> {
        Some(self)
    }
}
