module;

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <print>
#include <ranges>
#include <span>
#include <stdexcept>
#include <utility>

export module object;

import math;

namespace fs = std::filesystem;

class Material {
public:
  vec diffuse = vec(.5, .5, .5);
  vec specular = vec(1, 1, 1);
  vec emission = vec(0, 0, 0);
  scalar shininess = 20;
  scalar index_of_refraction = 1;

  Material() = default;
  Material(vec diffuse, vec specular, scalar shininess)
      : diffuse{diffuse}, specular{specular}, shininess{shininess} {}
};

#define EPSILON 1e-3

class AABB {
public:
  vec min, max;

  AABB() = default;
  AABB(vec a, vec b) : min{a}, max{b} {}

  void grow(AABB box) {
    min = vec(std::min(min.x, box.min.x), std::min(min.y, box.min.y),
              std::min(min.z, box.min.z));
    max = vec(std::max(max.x, box.max.x), std::max(max.y, box.max.y),
              std::max(max.z, box.max.z));
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
        if (n < 0)
          std::swap(t0, t1);
        if (t0 > tmin)
          tmin = t0;
        if (t1 < tmax)
          tmax = t1;
      }
    }
    return tmax >= tmin;
  }
};

export class Triangle;
export struct HitData {
  scalar t;
  const Triangle *obj;
  int hits = 0; // for special visualizations
};

export class Object {
public:
  Material *mat; // TODO: i'm not sure what's happening here
  Object(Material *mat = new Material()) : mat{mat} {}

  // all objects must have a bounding box (for BVH) and be intersectable
  virtual HitData intersect(vec origin, vec direction) const = 0;
  virtual AABB bbox() const = 0;
  virtual double area() const = 0;
  virtual std::tuple<vec, vec> sample() const = 0; // sample random point on surface of object, for NEE.
                      // returns: {point, normal}.

  vec centroid() const { return .5 * bbox().min + .5 * bbox().max; }
};

export class BVH {
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
  std::vector<Object *> objects;

  BVH() = default;
  BVH(auto objs) : objects{objs} {
    assert(!objects.empty());

    // create first node that spans entire object list
    nodes.push_back({.first = 0, .count = (unsigned int)objects.size()});

    // then recursively subdivide this node to create the tree
    update_box(0);
    subdivide(0);
  }

  HitData intersect(vec origin, vec direction, int i = 0) const {
    if (objects.size() == 0)
      return {INF};
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
    if (!nodes[i].box.intersect(origin, direction))
      return {INF};

    HitData lhit = intersect(origin, direction, nodes[i].index);
    HitData rhit = intersect(origin, direction, nodes[i].index + 1);
    ++lhit.hits;
    ++rhit.hits;
    return lhit.t < rhit.t ? lhit : rhit;
  }

  AABB bbox() const {
    if (objects.size() == 0)
      return AABB();
    return nodes[0].box;
  }

private:
  void update_box(int i) {
    nodes[i].box = AABB(vec(INF, INF, INF), vec(-INF, -INF, -INF));
    for (const auto &obj : nodes[i].span(objects)) {
      auto omin = obj->bbox().min;
      auto omax = obj->bbox().max;
      nodes[i].box.min = vec(std::min(nodes[i].box.min.x, omin.x),
                             std::min(nodes[i].box.min.y, omin.y),
                             std::min(nodes[i].box.min.z, omin.z));
      nodes[i].box.max = vec(std::max(nodes[i].box.max.x, omax.x),
                             std::max(nodes[i].box.max.y, omax.y),
                             std::max(nodes[i].box.max.z, omax.z));
    }
  }

  void subdivide(int i) {
    if (nodes[i].count <= max_leaf_size)
      return;

    unsigned int midpoint;

    // Estimate axis as one with greatest span.
    int axis = 0;
    vec extent = nodes[i].box.extent();
    if (extent.y > extent.x)
      axis = 1;
    if (extent.z > extent[axis])
      axis = 2;

    // Try to split it down the middle.
    auto start = objects.begin() + nodes[i].first;
    scalar mid = nodes[i].box.min[axis] + extent[axis] / 2;
    auto iter = std::partition(
        start, start + nodes[i].count,
        [axis, mid](const auto &obj) { return obj->centroid()[axis] < mid; });
    midpoint = std::distance(start, iter);

    // If we can't do that fall back to sorting it into two piles.
    if (midpoint == 0 || midpoint >= nodes[i].count - 1) {
      // O(n) compared to std::sort's O(n log n).
      std::nth_element(start, start + nodes[i].count / 2,
                       start + nodes[i].count,
                       [axis](const auto &a, const auto &b) {
                         return a->centroid()[axis] < b->centroid()[axis];
                       });
      midpoint = nodes[i].count / 2;
    }

    // Create children, update boxes, and recurse.
    int lhs = nodes.size();
    nodes.push_back({.first = nodes[i].first, .count = midpoint + 1});
    nodes.push_back({.first = nodes[i].first + midpoint + 1,
                     .count = nodes[i].count - (midpoint + 1)});
    nodes[i].index = lhs;
    nodes[i].count = 0;

    update_box(lhs);
    update_box(lhs + 1);
    subdivide(lhs);
    subdivide(lhs + 1);
  }
};

class Triangle : public Object {
public:
  Triangle(vec v0, vec v1, vec v2, vec vn0, vec vn1, vec vn2, bool smooth,
           Material *mat)
      : Object(mat), v0{v0}, v1{v1}, v2{v2}, vn0{vn0}, vn1{vn1}, vn2{vn2},
        smooth{smooth} {

    box = AABB(vec(std::min({v0.x, v1.x, v2.x}), std::min({v0.y, v1.y, v2.y}),
                   std::min({v0.z, v1.z, v2.z})),
               vec(std::max({v0.x, v1.x, v2.x}), std::max({v0.y, v1.y, v2.y}),
                   std::max({v0.z, v1.z, v2.z})));
  }

  vec normal() const { return (v1 - v0).cross(v2 - v0); }
  double area() const override {
    // Shoelace formula
    // clang-format off
    return 1/2 * std::abs(
      v0.x * (v1.y - v2.y) +
      v1.x * (v2.y - v0.y) +
      v2.x * (v0.y - v1.y)
    );
    // clang-format on
  }

  HitData intersect(vec origin, vec direction) const override {
    // Implements Möller-Trumbore.
    vec e1 = v1 - v0;
    vec e2 = v2 - v0;
    vec p = direction.cross(e2);
    scalar n = p.dot(e1);
    if (n == 0)
      return {INF};
    vec t = origin - v0;

    scalar u = 1 / n * p.dot(t);
    if (u < 0 || u > 1)
      return {INF};

    vec q = 1 / n * t.cross(e1);
    scalar v = q.dot(direction);
    if (v < 0 || u + v > 1)
      return {INF};
    return {q.dot(e2), this};
  }

  AABB bbox() const override { return box; }
  std::tuple<vec, vec> sample() const override {
    vec u = v1 - v0;
    vec v = v2 - v0;

    double s = randreal(0, 1);
    double t = randreal(0, 1);
    if (s + t <= 1) { // in triangle?
      return {s * u + t * v, normal()};
    }
    return {(1 - s) * u + (1 - t) * v, normal()};
  }

private:
  vec v0, v1, v2;
  vec vn0, vn1, vn2;
  bool smooth;
  AABB box;
};

class Polygon : public Object {
public:
  std::vector<vec> vertices, vnormals;
  std::vector<std::vector<std::array<unsigned int, 2>>> faces;
  bool smooth = false;
  double area_;
  std::vector<Triangle *> triangles;

  // This function should be called each time `faces` vector is modified.
  void init() {
    // Triangularize polygon.
    // TODO only works for convex polygons.
    auto addtriangle = [&](std::array<unsigned int, 2> v1,
                           std::array<unsigned int, 2> v2,
                           std::array<unsigned int, 2> v3) {
      triangles.push_back(new Triangle(
          vertices[v1[0]], vertices[v2[0]], vertices[v3[0]],
          v1[1] < vnormals.size() ? vnormals[v1[1]] : vertices[v1[0]],
          v2[1] < vnormals.size() ? vnormals[v2[1]] : vertices[v2[0]],
          v3[1] < vnormals.size() ? vnormals[v3[1]] : vertices[v3[0]], smooth,
          mat));
    };
    decltype(faces) newfaces;
    for (const auto &face : faces) {
      if (face.size() > 3) {
        auto pivot = face[0];
        for (size_t i = 1; i < face.size() - 1; ++i) {
          newfaces.push_back({pivot, face[i], face[i + 1]});
          addtriangle(pivot, face[i], face[i + 1]);
        }
      } else {
        newfaces.push_back({face[0], face[1], face[2]});
        addtriangle(face[0], face[1], face[2]);
      }
    }
    faces = newfaces;

    for (const auto *t : triangles) {
      area_ += t->area();
    }

    std::vector<Object *> objs;
    objs.reserve(triangles.size());
    for (Triangle *ptr : triangles) {
      objs.push_back(static_cast<Object *>(ptr));
    }

    // Reconstruct BVH.
    bvh = BVH(objs);
  }

  Polygon() = default;

  AABB bbox() const override { return bvh.bbox(); }
  HitData intersect(vec origin, vec direction) const override {
    return bvh.intersect(origin, direction);
  }
  double area() const override { return area_; }

  std::tuple<vec, vec> sample() const override {
    double r = randreal(0, area());
    double accum = 0;
    for (auto *t : triangles) {
      accum += t->area();
      if (r <= accum) {
        return t->sample();
      }
    }
    std::unreachable();
  }

private:
  BVH bvh;
};

export std::vector<Object *> parseobj(const std::string &filename) {
  std::println("started parsing {}", filename);
  auto split = [](const std::string &s, const std::string &delim = " ") {
    std::vector<std::string> toks = {};
    size_t i = 0, pi = 0;
    while ((i = s.find(delim, pi)) != std::string::npos) {
      // skip empty strings
      // if (i-pi != 0) {
      toks.push_back(s.substr(pi, i - pi));
      // }
      pi = i + delim.size();
    }
    toks.push_back(s.substr(pi));
    return toks;
  };

  auto parsemtl = [&split](const std::string &filename, auto &materials) {
    int lineno = 0;
    try {
      std::ifstream ifs(filename);
      if (!ifs)
        assert(false);

      Material *m;
      std::string line;
      while (std::getline(ifs, line)) {
        ++lineno;
        std::vector<std::string> toks = split(line);

        if (toks[0] == "newmtl") {
          assert(toks.size() == 2);
          m = new Material();
          materials[toks[1]] = m;
        } else if (toks[0] == "Kd") {
          assert(toks.size() == 4);
          m->diffuse =
              vec(std::stof(toks[1]), std::stof(toks[2]), std::stof(toks[3]));
        } else if (toks[0] == "Ke") {
          assert(toks.size() == 4);
          m->emission =
              vec(std::stof(toks[1]), std::stof(toks[2]), std::stof(toks[3]));
        } else if (toks[0] == "Ks") {
          assert(toks.size() == 4);
          m->specular =
              vec(std::stof(toks[1]), std::stof(toks[2]), std::stof(toks[3]));
        } else if (toks[0] == "Ns") {
          assert(toks.size() == 2);
          m->shininess = std::stof(toks[1]);
        } else if (toks[0] == "Ni") {
          assert(toks.size() == 2);
          m->index_of_refraction = std::stof(toks[1]);
        }
      }
    } catch (std::invalid_argument &e) {
      throw std::runtime_error(filename + ": " + std::to_string(lineno) +
                               ": parse error");
    }
  };

  int lineno = 0;
  try {
    std::ifstream ifs(filename);
    if (!ifs)
      assert(false);

    std::map<std::string, Material *> materials;
    std::vector<Object *> objects;
    int offset = 0;
    Polygon *p = nullptr;
    std::string line;
    while (std::getline(ifs, line)) {
      ++lineno;
      std::vector<std::string> toks = split(line);

      if (toks[0] == "o") {
        if (p != nullptr) {
          p->init();
          objects.push_back(p);
          offset += p->vertices.size();
        }
        p = new Polygon();
        continue;
      } else if (toks[0] == "s") {
        if (p == nullptr)
          p = new Polygon();
        assert(toks.size() == 2);
        if (toks[1] == "on") {
          p->smooth = true;
          continue;
        } else if (toks[1] == "off") {
          p->smooth = false;
          continue;
        }
        p->smooth = std::stoi(toks[1]) == 1;
      } else if (toks[0] == "v") {
        if (p == nullptr)
          p = new Polygon();
        assert(toks.size() == 4);
        p->vertices.push_back(
            vec(std::stof(toks[1]), std::stof(toks[2]), std::stof(toks[3])));
      } else if (toks[0] == "vn") {
        if (p == nullptr)
          p = new Polygon();
        assert(toks.size() == 4);
        p->vnormals.push_back(
            vec(std::stof(toks[1]), std::stof(toks[2]), std::stof(toks[3])));
      } else if (toks[0] == "f") {
        if (p == nullptr)
          p = new Polygon();
        std::vector<std::array<unsigned int, 2>> vs;
        for (const auto &vertex : toks | std::views::drop(1)) {
          std::vector<std::string> toks = split(vertex, "/");
          assert(toks.size() >= 2 && toks.size() <= 4);
          vs.push_back({(unsigned int)std::stoi(toks[0]) - 1 - offset,
                        toks.size() >= 3
                            ? (unsigned int)std::stoi(toks[2]) - 1 - offset
                            : 0});
        }
        p->faces.push_back({vs});
      } else if (toks[0] == "mtllib") {
        assert(toks.size() == 2);
        parsemtl(fs::path(filename).parent_path() / toks[1], materials);
      } else if (toks[0] == "usemtl") {
        if (p == nullptr)
          p = new Polygon();
        assert(toks.size() == 2);
        p->mat = materials[toks[1]];
      }
    }
    if (p != nullptr) {
      p->init();
      objects.push_back(p);
    }
    std::println("done parsing {}", filename);
    return objects;
  } catch (std::invalid_argument &e) {
    throw std::runtime_error(filename + ": " + std::to_string(lineno) +
                             ": parse error");
  }
}
