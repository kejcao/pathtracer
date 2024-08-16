module;

#include "progress.h"

#include <functional>
#include <math.h>
#include <random>
#include <thread>

// Loop from start (inclusive) to end (exclusive).
void parallel_for(int start, int end, std::function<void(int)> f,
                  int threadcnt = std::thread::hardware_concurrency()) {
  std::mutex m;
  auto runner = [&]() {
    for (;;) {
      int i;
      {
        std::lock_guard<std::mutex> lock(m);
        if ((i = start++) >= end)
          return;
        // std::cout << "\r" << i*100 / end << "%";
        // std::cout.flush();
      }
      f(i);
    }
  };
  std::vector<std::thread> threads;
  // threadcnt - 1 since main thread counts as one too.
  threadcnt = std::min(threadcnt - 1, end - start - 1);
  for (int i = 0; i < threadcnt; ++i)
    threads.push_back(std::thread(runner));
  runner();
  for (auto &&t : threads)
    t.join();
  // std::cout << "\r";
}

export module camera;

import math;
import scene;
import object;

export class Camera {
public:
  int threadcnt = std::thread::hardware_concurrency();

  int OUTW = 256, OUTH = 256;
  vec bgcolor = vec(0, 0, 0);
  vec pixels[256][256];

  int vw = 1, vh = 1;
  vec origin, direction;

  scalar fov = 65;

  Camera(const vec &origin, const vec &direction = vec(0, 0, 0))
      : origin{origin}, direction{direction} {}

  vec pathtrace(const Scene &scene, vec origin, vec direction, int depth = 0) {
    HitData hit = scene.castray(origin, direction);
    if (hit.t == INF)
      return bgcolor; // nothing hit?

    // return random direction on the hemisphere aligned with the normal
    // (assume normal is normalized?)
    auto random_direction = [](vec normal) {
      auto random_unit_sphere_direction = []() {
        scalar x = randreal(0, 1);
        scalar y = randreal(0, 1);
        scalar phi = 1 - 2 * x;
        scalar theta = std::fmax(0, 1 - phi * phi);
        return vec(cos(2 * M_PI * y) * sqrt(theta),
                   sin(2 * M_PI * y) * sqrt(theta), phi)
            .normalize();
      };

      vec v;
      do {
        v = random_unit_sphere_direction();
      } while (v.dot(normal) < 0);
      return v;
    };

    vec color = vec(0, 0, 0);

    // russian roulette!!!
    constexpr double probability_of_continuing = .7;
    if (depth <= 2 || randreal(0, 1) < probability_of_continuing) {
      auto mat = hit.obj->mat;
      vec brdf = mat->diffuse / M_PI;
      vec v = random_direction((hit.obj)->normal());

      // if (randreal(0, 1) < .5) {
      //   auto *o = scene.light_source();
      //   v = o;
      // }

      // if not in hemisphere around normal, flip it.
      // TODO normal should be to the outside of mesh.
      // if (hit.normal.dot(v) > 0)
      //   v = -v;

      vec intersection = origin + direction * hit.t;
      // corresponds to rendering equation.
      color += mat->emission + 2 * M_PI * brdf;
      color += ( // the irradiance
                   std::abs(direction.normalize().dot(v)) *
                   pathtrace(scene, intersection, v, depth + 1)) /
               probability_of_continuing;
    }
    return color;
  }

  vec point_towards(int px, int py) {
    return vec((px - (scalar)OUTW / 2) * vw / OUTW,
               // We negate this so positive Y goes up and negative Y down in
               // camera origin.
               (py - (scalar)OUTH / 2) * -vh / OUTH,
               // See https://en.wikipedia.org/wiki/Field_of_view#Photography
               1 / (2 * std::tan(fov / 2 * (M_PI / 180)) / vw))
        .rotate(direction);
  }

  void render(const Scene &scene, int samples = 32,
              std::function<void(int)> cb = nullptr) {

    Progress p = Progress(OUTH);

    // clear previous pixels buffer
    for (int py = 0; py < OUTH; ++py)
      for (int px = 0; px < OUTW; ++px)
        pixels[py][px] = vec(0, 0, 0);

    // start pathtracing!!! divvy up each row of pixels to different threads.
    parallel_for(
        0, OUTH,
        [&](int py) {
          for (int px = 0; px < OUTW; ++px) {
            vec dir = point_towards(px, py);
            for (int i = 0; i < samples; ++i) {
              pixels[py][px] += pathtrace(scene, origin, dir);
            }
            pixels[py][px] *= 255;
            pixels[py][px] /= samples;
            pixels[py][px] = vec(std::clamp((int)pixels[py][px].x, 0, 255),
                                 std::clamp((int)pixels[py][px].y, 0, 255),
                                 std::clamp((int)pixels[py][px].z, 0, 255));
          }
          p.increment();

          if (cb != nullptr) {
            cb(py); // this causes memory errors?
          }
        },
        threadcnt);
  }

  // just output basic PPM file, for debugging maybe. main interface is through
  // the QT GUI.
  void save(const std::string &filename) {
    std::ofstream ofs(filename);

    ofs << "P3\n";
    ofs << OUTW << " " << OUTH << "\n";
    ofs << 255 << "\n";

    int x, y;
    for (y = 0; y < OUTH; y++) {
      for (x = 0; x < OUTW; x++) {
        ofs << (int)pixels[y][x].x << " " << (int)pixels[y][x].y << " "
            << (int)pixels[y][x].z << "\n";
      }
    }
  }
};
