#ifndef OBJECTS_H
#define OBJECTS_H

#include <array>
#include "math.h"
#include "materials.h"

class Object;
struct HitData {
    double t;
    vec normal;
    const Object *obj;
};

class Object {
public:
    Material *mat;
    Object(Material *mat=new Material()) : mat{mat} {}
    virtual ~Object() = default; // TODO clean up mat
    virtual HitData intersect(vec origin, vec direction) const = 0;
};

// class Sphere : public Object {
// public:
//     double radius;

//     Sphere(const vec &center, double radius) : Object(center), radius{radius} { }

//     double intersect(vec origin, vec direction) const override {
//         vec v = origin - this->origin;
//         double a = direction.dot(direction);
//         double b = 2 * direction.dot(v);
//         double c = v.dot(v) - radius*radius;
//         double discriminant = b*b - 4*a*c;
//         return discriminant < 0 ? INF : (-b - sqrt(discriminant)) / (2*a);
//     }
// private:
// };

class Triangle : public Object {
public:
    // Triangle(
    //     vec v0, vec v1, vec v2
    // ) : v0{v0}, v1{v1}, v2{v2} {
    //     normal = (v1-v0).cross(v2-v0);
    // }

    Triangle(
        vec v0, vec v1, vec v2,
        vec vn0, vec vn1, vec vn2,
        bool smooth, Material *mat
    ) : Object(mat), v0{v0}, v1{v1}, v2{v2},
        vn0{vn0}, vn1{vn1}, vn2{vn2}, smooth{smooth} {
        normal = (v1-v0).cross(v2-v0);
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
private:
    vec v0, v1, v2;
    vec vn0, vn1, vn2;
    vec normal;
    bool smooth = false;
};

class Polygon : public Object {
public:
    using Vertex = std::array<unsigned int, 2>;
    std::vector<vec> vertices, vnormals;
    bool smooth = false;

    Polygon() = default;

    void addfaces(const std::vector<std::vector<std::array<unsigned int, 2>>> &faces) {
        auto addtriangle = [=, this](
            std::array<unsigned int, 2> v1,
            std::array<unsigned int, 2> v2,
            std::array<unsigned int, 2> v3
        ) {
            triangles.push_back(Triangle(
                vertices[v1[0]], vertices[v2[0]], vertices[v3[0]],
                v1[1] < vnormals.size() ? vnormals[v1[1]] : vertices[v1[0]],
                v2[1] < vnormals.size() ? vnormals[v2[1]] : vertices[v2[0]],
                v3[1] < vnormals.size() ? vnormals[v3[1]] : vertices[v3[0]],
                smooth, this->mat
            ));
        };

        for (const auto &face : faces) {
            if (face.size() > 3) {
                std::array<unsigned int, 2> pivot = face[0];
                for (size_t i = 1; i < face.size()-1; ++i) {
                    this->faces.push_back({pivot, face[i], face[i+1]});
                    addtriangle(pivot, face[i], face[i+1]);
                }
            } else {
                this->faces.push_back({face[0], face[1], face[2]});
                addtriangle(face[0], face[1], face[2]);
            }
        }
    }

    HitData intersect(vec origin, vec direction) const override {
        HitData closest = {INF};
        for (const auto &triangle : triangles) {
            HitData hit = triangle.intersect(origin, direction);
            if (hit.t < closest.t) {
                closest = hit;
                // std::cout << hit.obj->mat->diffuse << std::endl;
                // std::cout << triangle.mat->diffuse << std::endl;
            }
        }
        return closest;
    }
private:
    std::vector<std::array<std::array<unsigned int, 2>, 3>> faces;
    std::vector<Triangle> triangles;
};

#endif