use std::f64::consts::PI;

use lerp::Lerp;

use crate::pathfinding::triangulate::triangulate;

use super::{
    geom::{Arc, EnvPolygon, Segment},
    math::{self, Vec2f},
};

pub struct Circle {
    pub position: Vec2f,
    pub radius: f64,
}

pub struct Polygon {
    pub vertices: Vec<Vec2f>,
}

pub enum Obstacle {
    Circle(Circle),
    Polygon(Polygon),
}

impl Obstacle {
    pub fn convert_into_env(&self, inflate: f64) -> EnvPolygon {
        match self {
            Obstacle::Circle(c) => convert_circle(c, inflate),
            Obstacle::Polygon(p) => convert_polygon(p, inflate),
        }
    }
}

fn convert_circle(circle: &Circle, inflate: f64) -> EnvPolygon {
    EnvPolygon {
        arcs: vec![Arc {
            center: circle.position,
            radius: circle.radius + inflate,
            min_angle: -PI,
            max_angle: -PI,
        }],
        segments: vec![],
        inner_region: vec![],
        inverted: false,
    }
}

fn convert_polygon(polygon: &Polygon, inflate: f64) -> EnvPolygon {
    struct RegionElem {
        segment: Segment,
        arc: Option<Arc>,
    }

    let size = polygon.vertices.len();

    let mut elements = Vec::new();

    let mut prev = polygon.vertices[size - 1];
    for i in 0..size {
        let vertex = polygon.vertices[i];
        let delta = vertex - prev;

        let scale = inflate / delta.length();
        let offset = Vec2f {
            x: delta.y * scale,
            y: -delta.x * scale,
        };

        let from = vertex + offset;
        let to = prev + offset;

        let next = polygon.vertices[(i + 1) % size];
        let edge_angle = delta.angle();
        let next_angle = (next - vertex).angle();

        // Only add arc if the vertex is convex
        let arc = if math::floor_mod(next_angle - edge_angle, PI * 2.0) < PI {
            Some(Arc::new(
                vertex,
                inflate,
                edge_angle - PI / 2.0,
                next_angle - PI / 2.0,
            ))
        } else {
            None
        };

        elements.push(RegionElem {
            segment: Segment { from, to },
            arc,
        });

        prev = vertex;
    }

    // Concave vertices will generate overlapping segments, so we need to
    // remove those overlaps
    for i in 0..size {
        let i2 = (i + 1) % size;

        let a = &elements[i].segment;
        let b = &elements[i2].segment;

        let d1 = a.to - a.from;
        let d2 = b.to - b.from;

        let vp = d1.x * d2.y - d2.x * d1.y;
        let v = b.from - a.from;

        let k1 = (v.x * d2.y - v.y * d2.x) / vp;
        let k2 = (v.x * d1.y - v.y * d1.x) / vp;

        if vp != 0.0 && 0.0 <= k1 && k1 <= 1.0 && 0.0 <= k2 && k2 <= 1.0 {
            // Segments overlap, clip them
            let intersect = a.from.lerp(a.to, k1);
            elements[i].segment.from = intersect;
            elements[i2].segment.to = intersect;
        }
    }

    // Check winding order
    let first = polygon.vertices[0];
    let last = polygon.vertices.last().unwrap();
    let mut total = last.x * first.y - first.x * last.y;
    for i in 0..(size - 1) {
        let a = polygon.vertices[i];
        let b = polygon.vertices[i + 1];
        total += a.x * b.y - b.x * a.y;
    }
    let inverted = total < 0.0;

    let mut segments = Vec::new();
    let mut arcs = Vec::new();
    let mut inner_vertices = Vec::new();
    for elem in elements {
        inner_vertices.push(elem.segment.to);
        if let Some(arc) = &elem.arc {
            inner_vertices.push(elem.segment.from);
            inner_vertices.push(arc.center);
        }

        segments.push(elem.segment);
        if let Some(arc) = elem.arc {
            arcs.push(arc);
        }
    }

    // Triangulator requires vertices to be counter-clockwise
    if inverted {
        inner_vertices.reverse();
    }

    EnvPolygon {
        arcs,
        segments,
        inner_region: triangulate(&inner_vertices),
        inverted,
    }
}
