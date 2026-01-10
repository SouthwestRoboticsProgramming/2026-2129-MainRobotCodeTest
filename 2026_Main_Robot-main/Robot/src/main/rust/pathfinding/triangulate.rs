// Triangulation based on https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf

use itertools::Itertools;

use super::math::Vec2f;

pub struct Triangle(pub Vec2f, pub Vec2f, pub Vec2f);

impl Triangle {
    pub fn contains(&self, point: Vec2f) -> bool {
        // From https://stackoverflow.com/a/2049593

        fn sign(p1: Vec2f, p2: Vec2f, p3: Vec2f) -> f64 {
            (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)
        }

        let d1 = sign(point, self.0, self.1);
        let d2 = sign(point, self.1, self.2);
        let d3 = sign(point, self.2, self.0);

        let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
        let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;

        !(has_neg && has_pos)
    }
}

struct Vertex {
    position: Vec2f,
    prev_idx: usize,
    next_idx: usize,
}

fn get_tri(vertices: &Vec<Vertex>, vertex: &Vertex) -> Triangle {
    Triangle(
        vertices[vertex.prev_idx].position,
        vertex.position,
        vertices[vertex.next_idx].position,
    )
}

fn is_convex(vertices: &Vec<Vertex>, vertex: &Vertex) -> bool {
    let Triangle(prev, curr, next) = get_tri(vertices, vertex);

    let det = (curr.x - prev.x) * (next.y - curr.y) - (next.x - curr.x) * (curr.y - prev.y);
    det > 0.0
}

fn is_ear(vertices: &Vec<Vertex>, reflex: &Vec<usize>, vertex: &Vertex) -> bool {
    let tri = get_tri(vertices, vertex);

    for &reflex_idx in reflex {
        if reflex_idx == vertex.prev_idx || reflex_idx == vertex.next_idx {
            continue;
        }

        if tri.contains(vertices[reflex_idx].position) {
            return false;
        }
    }

    true
}

// Partitions a possibly-concave polygon into a set of triangles
// The points must be specified in counter-clockwise order (inside is to the left)
pub fn triangulate(points: &[Vec2f]) -> Vec<Triangle> {
    let mut vertices = points
        .iter()
        .enumerate()
        .map(|(idx, position)| Vertex {
            position: *position,
            prev_idx: if idx == 0 { points.len() - 1 } else { idx - 1 },
            next_idx: if idx == points.len() - 1 { 0 } else { idx + 1 },
        })
        .collect_vec();

    let mut convex = Vec::new();
    let mut reflex = Vec::new();
    for i in 0..vertices.len() {
        if is_convex(&vertices, &vertices[i]) {
            convex.push(i);
        } else {
            reflex.push(i);
        }
    }

    let mut ears = Vec::new();
    for &convex_idx in &convex {
        if is_ear(&vertices, &reflex, &vertices[convex_idx]) {
            ears.push(convex_idx);
        }
    }

    let triangle_count = points.len() - 2;
    let mut triangles = Vec::with_capacity(triangle_count);
    for _ in 0..triangle_count {
        let ear_idx = ears.remove(0);
        convex.retain(|&i| i != ear_idx);

        let v = &vertices[ear_idx];
        let curr_pos = v.position;
        let prev_idx = v.prev_idx;
        let next_idx = v.next_idx;

        let prev = &mut vertices[prev_idx];
        let prev_pos = prev.position;
        prev.next_idx = next_idx;

        let next = &mut vertices[next_idx];
        let next_pos = next.position;
        next.prev_idx = prev_idx;

        triangles.push(Triangle(prev_pos, curr_pos, next_pos));

        fn update_neighbor(
            vertices: &Vec<Vertex>,
            reflex: &mut Vec<usize>,
            ears: &mut Vec<usize>,
            index: usize,
        ) {
            let vertex = &vertices[index];
            if reflex.contains(&index) {
                if is_convex(vertices, vertex) {
                    reflex.retain(|&i| i != index);
                    if is_ear(vertices, reflex, vertex) {
                        ears.push(index);
                    }
                }
            } else if ears.contains(&index) && !is_ear(vertices, reflex, vertex) {
                ears.retain(|&i| i != index);
            }
        }
        update_neighbor(&vertices, &mut reflex, &mut ears, prev_idx);
        update_neighbor(&vertices, &mut reflex, &mut ears, next_idx);
    }

    triangles
}
