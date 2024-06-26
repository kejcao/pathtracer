#ifndef OBJECTS_H
#define OBJECTS_H

#include <array>
#include <iostream>
#include <algorithm>
#include <memory>
#include <span>
#include "math.h"
#include "materials.h"
#include "util.h"

#define EPSILON 1e-3

class AABB {
public:
    vec min, max;

    AABB() = default;
    AABB(vec a, vec b) : min{a}, max{b} { }

    void grow(AABB box) {
        min = vec(
            std::min(min.x, box.min.x),
            std::min(min.y, box.min.y),
            std::min(min.z, box.min.z)
        );
        max = vec(
            std::max(max.x, box.max.x),
            std::max(max.y, box.max.y),
            std::max(max.z, box.max.z)
        );
    }
    vec extent() const { return max - min; }
    double area() const {
        vec v = extent().square();
        return v.x * v.y * v.z;
    }

    bool intersect(vec origin, vec direction) const {
        scalar tmin = EPSILON, tmax = INF;
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
    int hits = 0;
};

class Object {
public:
    Material *mat; // TODO: i'm not sure what's happening here
    Object(Material *mat=new Material()) : mat{mat} {}

    // all objects must have a bounding box (for BVH) and be intersectable
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
        auto span(const auto &objs) const {
            assert(first + count <= objs.size());
            auto start = objs.begin() + first;
            return std::span(start, start + count);
        }
    };

    // tree is stored as a flat array
    std::vector<Node> nodes;
    std::vector<std::unique_ptr<Object>> objects;

    BVH() = default;
    BVH(auto objs) : objects{std::move(objs)} {
        assert (!objects.empty());

        // create first node that spans entire object list
        nodes.push_back({
            .first = 0,
            .count = (unsigned int)objects.size()
        });

        // then recursively subdivide this node to create the tree
        update_box(0);
        subdivide(0);
    }

    HitData intersect(vec origin, vec direction, int i=0) const {
        if (objects.size() == 0) return {INF};
        if (nodes[i].is_leaf()) {
            HitData closest = {INF};
            for (const auto &obj : nodes[i].span(objects)) {
                HitData hit = obj->intersect(origin, direction);
                if (hit.t >= EPSILON && hit.t < closest.t) {
                    ++hit.hits;
                    closest = hit;
                }
            }
            return closest;
        }
        if (!nodes[i].box.intersect(origin, direction)) return {INF};

        HitData lhit = intersect(origin, direction, nodes[i].index);
        HitData rhit = intersect(origin, direction, nodes[i].index+1);
        ++lhit.hits;
        ++rhit.hits;
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
        for (const auto &obj : nodes[i].span(objects)) {
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

        unsigned int midpoint;

        // Estimate axis as one with greatest span.
        int axis = 0;
        vec extent = nodes[i].box.extent();
        if (extent.y > extent.x) axis = 1;
        if (extent.z > extent[axis]) axis = 2;

        // Try to split it down the middle.
        auto start = objects.begin() + nodes[i].first;
        scalar mid = nodes[i].box.min[axis] + extent[axis] / 2;
        auto iter = std::partition(
            start, start + nodes[i].count,
            [axis, mid](const auto &obj) {
                return obj->centroid()[axis] < mid;
            }
        );
        midpoint = std::distance(start, iter);

        // If we can't do that fall back to sorting it into two piles.
        if (midpoint == 0 || midpoint >= nodes[i].count-1) {
            // O(n) compared to std::sort's O(n log n).
            std::nth_element(
                start, start + nodes[i].count/2, start + nodes[i].count,
                [axis](const auto &a, const auto &b) {
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
        vn0{vn0}, vn1{vn1}, vn2{vn2}, smooth{smooth} {

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
        // return {q.dot(e2), normal(), this};
        return {q.dot(e2), smooth ? (1-u-v)*v0 + u*v1 + v*v2 : normal(), this};
    }

    AABB bbox() const override { return box; }

private:
    vec v0, v1, v2;
    vec vn0, vn1, vn2;
    bool smooth;
    AABB box;
};

#include <assimp/mesh.h>
#include <assimp/scene.h>

class Polygon : public Object {
public:
    Polygon(const aiMesh* m, Material *mat) {
        // be wary: `m` will be deleted before Polygon destruction

        std::vector<vec> vertices, vnormals;
        std::vector<std::vector<unsigned int>> faces;
        bool smooth = false;

        auto *v = m->mVertices;
        for (size_t i = 0; i < m->mNumVertices; ++i) {
            vertices.push_back(vec(v[i].x, v[i].y, v[i].z));
        }

        auto *f = m->mFaces;
        for (size_t i = 0; i < m->mNumFaces; ++i) {
            assert (f[i].mNumIndices == 3);
            faces.push_back({f[i].mIndices[0], f[i].mIndices[1], f[i].mIndices[2]});
        }

        std::vector<std::unique_ptr<Object>> triangles;

        for (const auto &f : faces) {
            assert (f.size() == 3); // assume faces are triangles

            // TODO: Triangle seems space inefficient
            triangles.push_back(std::make_unique<Triangle>(
                vertices[f[0]], vertices[f[1]], vertices[f[2]],
                vec(0,0,0), vec(0,0,0), vec(0,0,0), smooth, mat
            ));
        }

        bvh = BVH(std::move(triangles));
    }

    ~Polygon() {
        std::cout << "started\n";
    }

    AABB bbox() const override { return bvh.bbox(); }
    HitData intersect(vec origin, vec direction) const override {
        return bvh.intersect(origin, direction);
    }

private:

    BVH bvh;
};

#endif
