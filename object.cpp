module;

#include "progress.h"
#include <algorithm>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <print>
#include <ranges>
#include <regex>
#include <span>
#include <stdexcept>
#include <utility>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

export module object;

import math;

namespace fs = std::filesystem;

export class Image {
public:
  AVFormatContext *fmt_ctx;
  AVCodecContext *dec_ctx;
  AVFrame *raw_frame;
  AVFrame *rgb_frame;
  AVPacket packet;
  SwsContext *sws_ctx;

  void error(bool cond, std::string msg) {
    if (cond) {
      std::println(stderr, "{}", msg);
      abort();
    }
  }

  // https://stackoverflow.com/questions/9652760/how-to-set-decode-pixel-format-in-libavcodec
  Image(const std::string &filename) {
    fmt_ctx = avformat_alloc_context();
    error(!fmt_ctx, "Could not allocate format context");
    error(avformat_open_input(&fmt_ctx, filename.c_str(), NULL, NULL) != 0,
          "Could not open image file");
    error(avformat_find_stream_info(fmt_ctx, NULL) < 0,
          "Could not find stream info");

    int vid_stream_index = -1;
    for (unsigned int i = 0; i < fmt_ctx->nb_streams; i++) {
      if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        vid_stream_index = i;
        break;
      }
    }
    error(vid_stream_index == -1, "Could not find a video stream");

    AVCodecParameters *codecParameters =
        fmt_ctx->streams[vid_stream_index]->codecpar;
    const AVCodec *codec = avcodec_find_decoder(codecParameters->codec_id);
    error(!codec, "Unsupported codec");

    // clang-format off
    // start initing everything
    dec_ctx = avcodec_alloc_context3(codec);
    error(!dec_ctx, "Could not allocate codec context");
    error(
      avcodec_parameters_to_context(dec_ctx, codecParameters) < 0,
      "Could not copy codec parameters to codec context"
    );
    error(avcodec_open2(dec_ctx, codec, NULL) < 0, "Could not open codec");

    int w = dec_ctx->width;
    int h = dec_ctx->height;
    raw_frame = av_frame_alloc();
    rgb_frame = av_frame_alloc();

    // setup sws to convert to AV_PIX_FMT_RGB24 format
    // TODO actually free this stuff. Not strictly necessary rn since we need
    // all the data until end of pathtracer run.
    uint8_t *buf = (uint8_t *)av_malloc(
        av_image_get_buffer_size(AV_PIX_FMT_RGB24, w, h, 1));
    av_image_fill_arrays(rgb_frame->data, rgb_frame->linesize, buf,
                         AV_PIX_FMT_RGB24, w, h, 1);

    rgb_frame->width = dec_ctx->width;
    rgb_frame->height = dec_ctx->height;
    rgb_frame->format = AV_PIX_FMT_RGB24;

    sws_ctx = sws_getContext(
      w, h, dec_ctx->pix_fmt, // src
      w, h, AV_PIX_FMT_RGB24, // dst
      SWS_BICUBIC, NULL, NULL, NULL
    );
    error(!sws_ctx, "Failed to initialize conversion context");

    // start reading pixel data
    error(av_read_frame(fmt_ctx, &packet) < 0, "Failed to read frame");
    error(packet.stream_index != vid_stream_index, "Video stream error");
    error(avcodec_send_packet(dec_ctx, &packet) < 0, "Failed to decode packet");

    int response = avcodec_receive_frame(dec_ctx, raw_frame);
    error(
      response < 0 ||
      response == AVERROR(EAGAIN) ||
      response == AVERROR_EOF,
      "Failed to decode frame"
    );

    sws_scale(sws_ctx,
      raw_frame->data, raw_frame->linesize, 0, dec_ctx->height,
      rgb_frame->data, rgb_frame->linesize
    );
    // clang-format on
  }

  int width() const { return raw_frame->width; }
  int height() const { return raw_frame->height; }

  vec at(int x, int y) const {
    assert(rgb_frame->format == AV_PIX_FMT_RGB24);
    assert(x < width() && y < height());

    uint8_t *v = rgb_frame->data[0] + (y * rgb_frame->linesize[0] + x * 3);
    return vec(v[0], v[1], v[2]) / 255;
  }

  ~Image() {
    sws_freeContext(sws_ctx);
    av_packet_unref(&packet);
    av_frame_free(&raw_frame);
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
  }
};

class Material {
public:
  vec diffuse = vec(.5, .5, .5);
  vec specular = vec(1, 1, 1);
  vec emission = vec(0, 0, 0);
  scalar shininess = 20;
  scalar index_of_refraction = 1;
  Image *diffuse_map = nullptr;

  Material() = default;
  Material(vec diffuse, vec specular, scalar shininess)
      : diffuse{diffuse}, specular{specular}, shininess{shininess} {}
};

#define EPSILON 1e-5

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
  vec barycentric;
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
  virtual std::tuple<vec, vec>
  sample() const = 0; // sample random point on surface of object, for NEE.
                      // returns: {point, normal}.

  vec centroid() const { return .5 * bbox().min + .5 * bbox().max; }
};

// A very convenient and crucial class that constructs a standard BVH tree from
// any list of Objects.
export class BVH {
public:
  unsigned int max_leaf_size = 8;

  struct Node {
    // nodes[index + 0] is left  child
    // nodes[index + 1] is right child
    union {
      // If count == 0 then we are an internal node and use index;
      // otherwise, we're a leaf and use first to point to objects.
      unsigned int index;
      unsigned int first;
    };
    unsigned int count; // Number of objects stored in this node.
    AABB box;

    bool is_leaf() const { return count > 0; }
    auto span(const auto &objs) const {
      assert(count != 0 && first + count <= objs.size());
      auto start = objs.begin() + first;
      return std::span(start, start + count);
    }
  };

  // BVH tree is stored as a flat array in nodes vector
  std::vector<Node> nodes;

  // Store list of original objects to perform intersection checks with
  std::vector<Object *> objects;

  BVH() = default;
  BVH(auto objs) : objects{objs} {
    assert(!objects.empty());

    // Create the first node which spans entire object list
    nodes.push_back({.first = 0, .count = (unsigned int)objects.size()});
    update_box(0);

    subdivide(0);
  }

  HitData intersect(vec origin, vec direction, int i = 0) const {
    if (nodes[i].is_leaf()) {
      // clang-format off
      return std::ranges::min(
        nodes[i].span(objects)
          | std::views::transform([&](const auto &obj) {
            return obj->intersect(origin, direction);
          }),
        {},
        [](const HitData &x) { return x.t; });
      // clang-format on
    } else {
      if (!nodes[i].box.intersect(origin, direction))
        return {INF};
      HitData lhit = intersect(origin, direction, nodes[i].index);
      HitData rhit = intersect(origin, direction, nodes[i].index + 1);
      ++lhit.hits;
      ++rhit.hits;
      return lhit.t < rhit.t ? lhit : rhit;
    }
  }

  AABB bbox() const {
    if (objects.size() == 0)
      return AABB();
    return nodes[0].box;
  }

private:
  // Given a node and the span of objects it's associated with, this function
  // sets the appropriate values for the AABB.
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

  // Given a leaf node, this function subdivides it into an internal node that
  // refers to two other leaf nodes, each comprising half of the objects of the
  // original leaf node.
  void subdivide(int i) {
    assert(nodes[i].count != 0); // ensure this is a leaf node
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

    // Update the AABBs of the new leaf nodes.
    update_box(lhs);
    update_box(lhs + 1);

    // Recursively subdivide the new leaf nodes.
    subdivide(lhs);
    subdivide(lhs + 1);
  }
};

struct Index {
  uint32_t i0;
  uint32_t i1;
  uint32_t i2;
};

class Polygon;
class Triangle : public Object {
public:
  Triangle(Polygon *parent, Index v, Index vt, Index vn, bool smooth,
           Material *mat)
      : Object(mat), parent{parent}, v{v}, vt{vt}, vn{vn}, smooth{smooth} {

    box = AABB(vec(std::min({v0().x, v1().x, v2().x}),
                   std::min({v0().y, v1().y, v2().y}),
                   std::min({v0().z, v1().z, v2().z})),
               vec(std::max({v0().x, v1().x, v2().x}),
                   std::max({v0().y, v1().y, v2().y}),
                   std::max({v0().z, v1().z, v2().z})));
  }

  vec v0() const;
  vec v1() const;
  vec v2() const;

  vec vt0() const;
  vec vt1() const;
  vec vt2() const;

  vec normal() const { return (v1() - v0()).cross(v2() - v0()); }

  double area() const override {
    // Shoelace formula
    // clang-format off
    return std::abs(
      v0().x * (v1().y - v2().y) +
      v1().x * (v2().y - v0().y) +
      v2().x * (v0().y - v1().y)
    ) / 2;
    // clang-format on
  }

  HitData intersect(vec origin, vec direction) const override {
    // Implements Möller-Trumbore.
    vec e1 = v1() - v0();
    vec e2 = v2() - v0();
    vec p = direction.cross(e2);
    scalar n = p.dot(e1);
    if (n == 0)
      return {INF};
    vec t = origin - v0();

    scalar u = 1 / n * p.dot(t);
    if (u < 0 || u > 1)
      return {INF};

    vec q = 1 / n * t.cross(e1);
    scalar v = q.dot(direction);
    if (v < 0 || u + v > 1)
      return {INF};

    scalar t_ = q.dot(e2);
    if (t_ < 1e-6) // distance too small indicates false positive.
      return {INF};
    return {t_, this, vec(u, v, 1 - u - v)};
  }

  AABB bbox() const override { return box; }
  std::tuple<vec, vec> sample() const override {
    vec u = v1() - v0();
    vec v = v2() - v0();

    double s = randreal(0, 1);
    double t = randreal(0, 1);
    if (s + t <= 1) { // in triangle?
      return {s * u + t * v, normal()};
    }
    return {(1 - s) * u + (1 - t) * v, normal()};
  }

private:
  Index v, vt, vn;
  bool smooth;
  AABB box;
  Polygon *parent;
};

class Polygon : public Object {
public:
  std::vector<vec> vertices, vnormals, vtextures;
  std::vector<std::vector<std::array<unsigned int, 3>>> faces;
  bool smooth = false;
  double area_;
  std::vector<Triangle *> triangles;

  // This function should be called each time `faces` vector is modified.
  void init() {
    std::println("v:{}, vt:{}, vn:{} f:{}", vertices.size(), vtextures.size(),
                 vnormals.size(), faces.size());

    // Triangularize polygon.
    // TODO only works for convex polygons.
    auto addtriangle = [&](std::array<unsigned int, 3> v1,
                           std::array<unsigned int, 3> v2,
                           std::array<unsigned int, 3> v3) {
      // clang-format off
      // in obj wavefront indexing begins at one, so we need to subtract by one.
      triangles.push_back(new Triangle(this,
          {v1[0], v2[0], v3[0]},
          {v1[1], v2[1], v3[1]},
          {v1[2], v2[2], v3[2]}, smooth, mat));
      // clang-format on
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

vec Triangle::v0() const {
  assert(v.i0 < parent->vertices.size());
  return parent->vertices[v.i0];
}
vec Triangle::v1() const {
  assert(v.i1 < parent->vertices.size());
  return parent->vertices[v.i1];
}
vec Triangle::v2() const {
  assert(v.i2 < parent->vertices.size());
  return parent->vertices[v.i2];
}

vec Triangle::vt0() const {
  assert(vt.i0 < parent->vtextures.size());
  return parent->vtextures[vt.i0];
}
vec Triangle::vt1() const {
  assert(vt.i1 < parent->vtextures.size());
  return parent->vtextures[vt.i1];
}
vec Triangle::vt2() const {
  assert(vt.i2 < parent->vtextures.size());
  return parent->vtextures[vt.i2];
}

auto parsemtl = [](fs::path filename, auto &materials) {
  Material *m;

  const std::vector<std::tuple<std::regex, std::function<void(std::smatch)>>> v{
      {std::regex("^\\s*newmtl\\s+(.*?)\\s*$"),
       [&](std::smatch matches) {
         m = new Material();
         materials[matches[1].str()] = m;
       }},
      {std::regex("^\\s*Kd\\s+([0-9.]+)\\s+([0-9.]+)\\s+([0-9.]+)\\s*$"),
       [&](std::smatch matches) {
         m->diffuse =
             vec(std::stof(matches[1].str()), std::stof(matches[2].str()),
                 std::stof(matches[3].str()));
       }},
      {std::regex("^\\s*Ke\\s+([0-9.]+)\\s+([0-9.]+)\\s+([0-9.]+)\\s*$"),
       [&](std::smatch matches) {
         m->emission =
             vec(std::stof(matches[1].str()), std::stof(matches[2].str()),
                 std::stof(matches[3].str()));
       }},
      {std::regex("^\\s*map_Kd\\s+(.*?)\\s*$"), [&](std::smatch matches) {
         m->diffuse_map =
             new Image(fs::path(filename).parent_path() / matches[1].str());
       }}};

  std::ifstream ifs(filename);
  std::string line;
  while (std::getline(ifs, line)) {
    for (const auto &[r, f] : v) {
      std::smatch matches;
      if (std::regex_match(line, matches, r)) {
        f(matches);
        break;
      }
    }
  }
};

// doesn't support obj wavefront files that "backreference" vertices.
export std::vector<Object *>
parseobj(const std::string &filename) { // this is shit code
  Progress pbar;

  veci offset = veci(0, 0, 0); // offset, for vertex counts. TODO
  Polygon *p = nullptr;        // current polygon object
  std::map<std::string, Material *> materials;
  std::vector<Object *> objects; // past polygon objects

  auto FACE = // talk about overcomplication... the cpp muse guided me towards
              // this code.
      [&](const std::string &re,
          std::function<veci(std::vector<unsigned int>)> f)
      -> std::tuple<std::regex, std::function<void(std::smatch)>> {
    std::regex re_ = std::regex(re); // precompile
    return {
        std::regex(std::format("^\\s*f({})+\\s*$", re)),
        [&, re_, re](std::smatch matches) {
          auto search = [&offset, &p](std::string text, std::regex pattern) {
            std::sregex_iterator it(text.begin(), text.end(), pattern);
            std::sregex_iterator end;
            std::vector<std::vector<unsigned int>> parts;
            while (it != end) {
              parts.push_back({});
              for (int i = 1; i < (*it).size(); ++i) {
                parts.back().push_back((unsigned int)std::stoi((*it)[i].str()));
              }
              ++it;
            }
            return parts;
          };

          auto v = search(matches[0].str(), re_);
          std::vector<std::array<unsigned int, 3>> face;
          for (const auto &l : v) {
            auto v = f(l);
            v -= veci(1, 1, 1);
            v -= offset;
            face.push_back(
                {(unsigned int)v.x, (unsigned int)v.y, (unsigned int)v.z});
          }
          p->faces.push_back(face);
        }};
  };

  // TODO implement s for smoothing
  const std::vector<std::tuple<std::regex, std::function<void(std::smatch)>>> v{
      {std::regex("^\\s*v\\s+(-?[0-9.]+)\\s+(-?[0-9.]+)\\s+(-?[0-9.]+)\\s*$"),
       [&](std::smatch matches) {
         assert(p != nullptr);
         p->vertices.push_back({
             std::stof(matches[1].str()),
             std::stof(matches[2].str()),
             std::stof(matches[3].str()),
         });
       }},
      {std::regex("^\\s*vn\\s+(-?[0-9.]+)\\s+(-?[0-9.]+)\\s+(-?[0-9.]+)\\s*$"),
       [&](std::smatch matches) {
         assert(p != nullptr);
         p->vnormals.push_back({
             std::stof(matches[1].str()),
             std::stof(matches[2].str()),
             std::stof(matches[3].str()),
         });
       }},
      {std::regex(
           "^\\s*vt\\s+(-?[0-9.]+)(\\s+-?[0-9.]+)?(\\s+-?[0-9.]+)?\\s*$"),
       [&](std::smatch matches) {
         assert(p != nullptr);
         p->vtextures.push_back({
             std::stof(matches[1].str()),
             matches[2].matched ? std::stof(matches[2].str()) : 0,
             matches[3].matched ? std::stof(matches[3].str()) : 0,
         });
       }},
      FACE("\\s+([0-9]+)\\/([0-9]+)\\/([0-9]+)",
           [](auto l) -> veci { return veci(l[0], l[1], l[2]); }),
      FACE("\\s+([0-9]+)\\/\\/([0-9]+)",
           [](auto l) -> veci { return veci(l[0], 0, l[1]); }),
      {std::regex("^o\\s(.*)$"),
       [&](std::smatch matches) {
         if (p != nullptr) {
           p->init();
           objects.push_back(p);
           offset += veci(p->vertices.size(), p->vtextures.size(),
                          p->vnormals.size());
         }
         p = new Polygon();
       }},
      {std::regex("^\\s*mtllib\\s+(.*?)\\s*$"),
       [&](std::smatch matches) {
         parsemtl(fs::path(filename).parent_path() / matches[1].str(),
                  materials);
       }},
      {std::regex("^\\s*usemtl\\s+(.*?)\\s*$"),
       [&](std::smatch matches) { p->mat = materials[matches[1].str()]; }}};

  std::ifstream ifs(filename);
  std::string line;
  while (std::getline(ifs, line)) {
    for (const auto &[r, f] : v) {
      std::smatch matches;
      if (std::regex_match(line, matches, r)) {
        f(matches);
        break;
      }
    }
  }

  if (p != nullptr) {
    p->init();
    objects.push_back(p);
  }
  pbar.finish(std::format("parse {}", filename));
  return objects;
}
