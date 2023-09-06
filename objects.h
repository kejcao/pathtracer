#ifndef OBJECTS_H
#define OBJECTS_H

#include <array>
#include <iostream>
#include <algorithm>
#include <span>
#include "math.h"
#include "materials.h"

class AABB {
public:
    vec min, max;

    AABB() = default;
    AABB(vec a, vec b) : min{a}, max{b} { }

    vec extent() const { return max - min; }

    bool intersect(vec origin, vec direction) const {
        scalar tmin = 1e-8, tmax = INF;
        for (int i = 0; i < 3; ++i) {
            if (direction[i] != 0) {
                scalar n = 1 / direction[i];
                scalar t0 = (min[i] - origin[i]) * n;
                scalar t1 = (max[i] - origin[i]) * n;
                if (n < 0) std::swap(t0, t1);
                if (t0 > tmin) tmin = t0;
                if (t1 < tmax) tmax = t1;
            }
        }
        return tmax >= tmin;
    }
};

class Object;
struct HitData {
    scalar t;
    vec normal;
    const Object *obj;
};

class Object {
public:
    Material *mat;
    Object(Material *mat=new Material()) : mat{mat} {}
    virtual ~Object() { delete mat; }

    virtual AABB bbox() const = 0;
    virtual HitData intersect(vec origin, vec direction) const = 0;
    vec centroid() const {
        return .5 * bbox().min + .5 * bbox().max;
    }
};

class BVH {
public:    
    unsigned int max_leaf_size = 8;

    struct Node {
        union {
            // If `count` == 0 then we are an internal node and use `index`;
            // otherwise, we're a leaf and use `first` to point to objects.
            unsigned int index; // `index` is left child, `index`+1 is right child.
            unsigned int first;
        };
        unsigned int count; // How many objects there are in this node.
        AABB box;

        bool is_leaf() const { return count > 0; }
        auto span(const std::vector<Object *> &objs) const {
            assert(first + count <= objs.size());
            auto start = objs.begin() + first;
            return std::span(start, start + count);
        }
    };

    std::vector<Object *> objects;
    std::vector<Node> nodes;

    BVH() = default;
    BVH(const std::vector<Object *> &objects) : objects{objects} {
        if (objects.size() != 0) {
            nodes.push_back({
                .first = 0,
                .count = (unsigned int)objects.size()
            });
            update_box(0);
            subdivide(0);
        }
    }
    ~BVH() {
        // TODO double free when we do this—why?
        // for (auto &&obj : objects) delete obj;
    }

    HitData intersect(vec origin, vec direction, int i=0) const {
        if (objects.size() == 0) return {INF};
        if (nodes[i].is_leaf()) {
            HitData closest = {INF};
            for (auto *obj : nodes[i].span(objects)) {
                HitData hit = obj->intersect(origin, direction);
                if (hit.t >= 1e-8 && hit.t < closest.t) {
                    closest = hit;
                }
            }
            return closest;
        }
        if (!nodes[i].box.intersect(origin, direction)) return {INF};

        HitData lhit = intersect(origin, direction, nodes[i].index);
        HitData rhit = intersect(origin, direction, nodes[i].index+1);
        return lhit.t < rhit.t ? lhit : rhit;
    }

    AABB bbox() const {
        if (objects.size() == 0) return AABB();
        return nodes[0].box;
    }

private:
    void update_box(int i) {
        nodes[i].box = AABB(
            vec( INF,  INF,  INF),
            vec(-INF, -INF, -INF)
        );
        for (auto *obj : nodes[i].span(objects)) {
            auto omin = obj->bbox().min;
            auto omax = obj->bbox().max;
            nodes[i].box.min = vec(
                std::min(nodes[i].box.min.x, omin.x),
                std::min(nodes[i].box.min.y, omin.y),
                std::min(nodes[i].box.min.z, omin.z)
            );
            nodes[i].box.max = vec(
                std::max(nodes[i].box.max.x, omax.x),
                std::max(nodes[i].box.max.y, omax.y),
                std::max(nodes[i].box.max.z, omax.z)
            );
        }
    }

    void subdivide(int i) {
        if (nodes[i].count <= max_leaf_size) return;

        int axis = 0;
        vec extent = nodes[i].box.extent();
        if (extent.y > extent.x) axis = 1;
        if (extent.z > extent[axis]) axis = 2;

        // Try to split it down the middle.
        auto start = objects.begin() + nodes[i].first;
        scalar mid = nodes[i].box.min[axis] + extent[axis] / 2;
        auto iter = std::partition(
            start, start + nodes[i].count,
            [axis, mid](auto *obj) {
                return obj->centroid()[axis] < mid;
            }
        );
        unsigned int midpoint = std::distance(start, iter);

        // If we can't do that fall back to sorting it into two piles.
        if (midpoint == 0 || midpoint >= nodes[i].count-1) {
            std::nth_element(
                start, start + nodes[i].count/2, start + nodes[i].count,
                [axis](auto *a, auto *b) {
                    return a->centroid()[axis] < b->centroid()[axis];
                }
            );
            midpoint = nodes[i].count / 2;
        }

        // Create children, update boxes, and recurse.
        int lhs = nodes.size();
        nodes.push_back({
            .first = nodes[i].first,
            .count = midpoint + 1
        });
        nodes.push_back({
            .first = nodes[i].first + midpoint+1,
            .count = nodes[i].count - (midpoint+1)
        });
        nodes[i].index = lhs;
        nodes[i].count = 0;

        update_box(lhs);
        update_box(lhs+1);
        subdivide(lhs);
        subdivide(lhs+1);
    }
};

class Triangle : public Object {
public:
    Triangle(
        vec v0, vec v1, vec v2,
        vec vn0, vec vn1, vec vn2,
        bool smooth, Material *mat
    ) : Object(mat), v0{v0}, v1{v1}, v2{v2},
        // vn0{vn0}, vn1{vn1}, vn2{vn2}, smooth{smooth} {
         smooth{smooth} {

        box = AABB(
            vec(
                std::min({v0.x, v1.x, v2.x}),
                std::min({v0.y, v1.y, v2.y}),
                std::min({v0.z, v1.z, v2.z})
            ),
            vec(
                std::max({v0.x, v1.x, v2.x}),
                std::max({v0.y, v1.y, v2.y}),
                std::max({v0.z, v1.z, v2.z})
            )
        );
    }

    vec normal() const { return (v1-v0).cross(v2-v0); }

    HitData intersect(vec origin, vec direction) const override {
        // Implements Möller-Trumbore.
        vec e1 = v1 - v0;
        vec e2 = v2 - v0;
        vec p = direction.cross(e2);
        scalar n = p.dot(e1);
        if (n == 0) return {INF};
        vec t = origin - v0;

        scalar u = 1/n * p.dot(t);
        if (u < 0 || u > 1) return {INF};

        vec q = 1/n * t.cross(e1);
        scalar v = q.dot(direction);
        if (v < 0 || u+v > 1) return {INF};
        // return {q.dot(e2), smooth ? (1-u-v)*vn0 + u*vn1 + v*vn2 : normal(), this};
        return {q.dot(e2), smooth ? (1-u-v)*v0 + u*v1 + v*v2 : normal(), this};
    }

    AABB bbox() const override { return box; }

private:
    vec v0, v1, v2;
    // vec vn0, vn1, vn2;
    bool smooth;
    AABB box;
};

class Polygon : public Object {
public:
    std::vector<vec> vertices, vnormals;
    std::vector<std::vector<std::array<unsigned int, 2>>> faces;
    bool smooth = false;
    std::vector<Object *> triangles;

    // This function should be called each time `faces` vector is modified.
    void init() {
        // Triangularize polygon.
        // TODO only works for convex polygons.
        auto addtriangle = [&](
            std::array<unsigned int, 2> v1,
            std::array<unsigned int, 2> v2,
            std::array<unsigned int, 2> v3
        ) {
            triangles.push_back(new Triangle(
                vertices[v1[0]], vertices[v2[0]], vertices[v3[0]],
                v1[1] < vnormals.size() ? vnormals[v1[1]] : vertices[v1[0]],
                v2[1] < vnormals.size() ? vnormals[v2[1]] : vertices[v2[0]],
                v3[1] < vnormals.size() ? vnormals[v3[1]] : vertices[v3[0]],
                smooth, mat
            ));
        };
        decltype(faces) newfaces;
        for (const auto &face : faces) {
            if (face.size() > 3) {
                auto pivot = face[0];
                for (size_t i = 1; i < face.size()-1; ++i) {
                    newfaces.push_back({pivot, face[i], face[i+1]});
                    addtriangle(pivot, face[i], face[i+1]);
                }
            } else {
                newfaces.push_back({face[0], face[1], face[2]});
                addtriangle(face[0], face[1], face[2]);
            }
        }
        faces = newfaces;

        // Reconstruct BVH.
        bvh = BVH(triangles);
    }

    Polygon() = default;

    AABB bbox() const override { return bvh.bbox(); }
    HitData intersect(vec origin, vec direction) const override {
        return bvh.intersect(origin, direction);
    }

private:
    BVH bvh;
};

#endif