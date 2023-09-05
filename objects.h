#ifndef OBJECTS_H
#define OBJECTS_H

#include <array>
#include <iostream>
#include <algorithm>
#include <span>
#include "math.h"
#include "materials.h"

unsigned long long bvh_rejected = 0, bvh_total = 0;

class Object;
struct HitData {
    double t;
    vec normal;
    const Object *obj;
};

class AABB {
public:
    vec min, max;

    AABB() = default;
    AABB(vec a, vec b) : min{a}, max{b} { }
    AABB(AABB a, AABB b) {
        min = vec(
            std::min(a.min.x, b.min.x),
            std::min(a.min.y, b.min.y),
            std::min(a.min.z, b.min.z)
        );
        max = vec(
            std::max(a.max.x, b.max.x),
            std::max(a.max.y, b.max.y),
            std::max(a.max.z, b.max.z)
        );
    }

    bool intersect(vec origin, vec direction) const {
        for (int i = 0; i < 3; ++i) {
            double n = 1 / direction[i];
            double t0 = (min[i] - origin[i]) * n;
            double t1 = (max[i] - origin[i]) * n;
            if (n < 0) std::swap(t0, t1);
            if (t1 < t0) return false;
            // if (t1 < t0 && t0 > 1e-8) return false;
        }
        return true;
    }
};

class Intersectable {
public:
    virtual AABB bbox() const = 0;
    vec centroid() const { return .5 * bbox().min + .5 * bbox().max; }
    virtual HitData intersect(vec origin, vec direction) const = 0;
    virtual ~Intersectable() = default;
};

class Object : public Intersectable {
public:
    Material *mat;
    Object(Material *mat=new Material()) : mat{mat} {}
    ~Object() { delete mat; }
};

class BVH : public Intersectable {
public:    
    AABB box;
    Intersectable *left, *right;

    BVH() = default;
    BVH(std::span<Object *> objs) {
        assert(objs.size() != 0);
        int axis = 1;
        auto cmp = [&](auto *a, auto *b) {
            return a->bbox().min[axis] < b->bbox().min[axis];
        };
        if (objs.size() == 1) {
            left = right = objs[0];
        } else if(objs.size() == 2) {
            left = objs[0];
            right = objs[1];
        } else {
            vec min, max;
            for (const auto *obj : objs) {
                min = min.min(obj->centroid());
                max = max.max(obj->centroid());
            }
            vec extent = max - min;
            if (extent.y > extent.x) axis = 1;
            if (extent.z > extent.x) axis = 2;

            std::sort(objs.begin(), objs.end(), cmp);

            left = new BVH(std::span(objs.begin(), objs.size()/2));
            right = new BVH(std::span(objs.begin() + objs.size()/2, objs.end()));
        }
        box = AABB(left->bbox(), right->bbox());
    }

    HitData intersect(vec origin, vec direction) const override {
        ++bvh_total;
        if (!box.intersect(origin, direction)) {
            ++bvh_rejected;
            return {INF};
        }
        HitData hitl = left->intersect(origin, direction);
        HitData hitr = right->intersect(origin, direction);
        if (hitl.t < 1e-8) hitl.t = INF;
        if (hitr.t < 1e-8) hitr.t = INF;
        if (hitr.t != INF) {
            return hitr.t < hitl.t ? hitr : hitl;
        }
        return hitl;
    }

    AABB bbox() const override {
        return box;
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

        normal = (v1-v0).cross(v2-v0);
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

    HitData intersect(vec origin, vec direction) const override {
        // Implements Möller-Trumbore.
        vec e1 = v1 - v0;
        vec e2 = v2 - v0;
        vec p = direction.cross(e2);
        double n = p.dot(e1);
        if (n == 0) return {INF};
        vec t = origin - v0;

        double u = 1/n * p.dot(t);
        if (u < 0 || u > 1) return {INF};

        vec q = 1/n * t.cross(e1);
        double v = q.dot(direction);
        if (v < 0 || u+v > 1) return {INF};
        return {q.dot(e2), smooth ? (1-u-v)*vn0 + u*vn1 + v*vn2 : normal, this};
    }

    AABB bbox() const override { return box; }

private:
    vec v0, v1, v2;
    vec vn0, vn1, vn2;
    vec normal;
    bool smooth;
    AABB box;
};

class Polygon : public Object {
public:
    std::vector<vec> vertices, vnormals;
    std::vector<std::vector<std::array<unsigned int, 2>>> faces;
    bool smooth = false;

    // This function should be called each time `faces` vector is modified.
    void init() {
        // Triangularize polygon.
        // TODO only works for convex polygons.
        for (auto &&t : triangles) delete t;
        triangles = {};
        auto addtriangle = [=, this](
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
        if (triangles.size() != 0) {
            std::vector<Object *> tmp(triangles.begin(), triangles.end());
            bvh = BVH(tmp);
        }
    }

    Polygon() = default;
    // ~Polygon() {
    //     for (auto &&t : triangles) delete t;
    // }

    HitData intersect(vec origin, vec direction) const override {
        if (triangles.size() == 0) return {INF};
        return bvh.intersect(origin, direction);
        HitData closest = {INF};
        for (const auto *t : triangles) {
            HitData hit = t->intersect(origin, direction);
            if (hit.t < closest.t) {
                closest = hit;
            }
        }
        return closest;
    }

    AABB bbox() const override {
        AABB box;
        for (const auto *t : triangles) {
            auto min = t->bbox().min;
            box.min = vec(
                std::min(box.min.x, min.x),
                std::min(box.min.y, min.y),
                std::min(box.min.z, min.z)
            );
            auto max = t->bbox().max;
            box.max = vec(
                std::max(box.max.x, max.x),
                std::max(box.max.y, max.y),
                std::max(box.max.z, max.z)
            );
        }
        return box;
    }

private:
    std::vector<Triangle *> triangles;
    BVH bvh;
};

#endif