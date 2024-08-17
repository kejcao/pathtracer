module;

#include "progress.h"

#include <cassert>
#include <functional>
#include <iostream>
#include <math.h>
#include <print>
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

    vec intersection = origin + direction * hit.t;
    vec normal = (hit.obj)->normal().normalize();

    // ensure normal is pointing outward of the mesh
    if (normal.dot(direction.normalize()) > 0) // wtf?
      normal = -normal;

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

      vec v = random_unit_sphere_direction();
      return v.dot(normal) < 0 ? -v : v;
    };

    vec color = vec(0, 0, 0);

    // direct light sampling aka NEE
    for (const Object *light : scene.lights()) {
      auto [point, light_normal] = light->sample();
      vec to_light = (point - intersection).normalize();

      // visibility ray; if we can't see the light source, skip.
      if (scene.castray(intersection, to_light).t == INF)
        continue;

      scalar cos_theta = to_light.dot(normal);
      if (cos_theta < 0)
        cos_theta = 0;

      scalar y = to_light.dot(light_normal);
      if (y < 0)
        y = 0;
      y /= std::pow(to_light.norm(), 2);

      color += light->mat->emission * cos_theta * y;
    }

    // rr_prob is russian roulette probability; aka probability of continuing.
    constexpr double rr_prob = .7;
    if (depth <= 2 || randreal(0, 1) < rr_prob) {
      vec brdf = hit.obj->mat->diffuse / M_PI; // lol
      vec randdir = random_direction(normal);

      // 1/2*pi in the PDF to account for hemisphere
      constexpr double pdf = 1 / (2 * M_PI) * rr_prob;

      color += hit.obj->mat->emission; // add the emitted radiance
      color += pathtrace(scene, intersection, randdir, depth + 1) * brdf *
               std::abs(normal.dot(randdir)) / pdf; // the rendering equation
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
              std::function<void()> cb = nullptr) {

    // Progress p = Progress(OUTH);

    // clear previous pixels buffer
    for (int py = 0; py < OUTH; ++py)
      for (int px = 0; px < OUTW; ++px)
        pixels[py][px] = vec(0, 0, 0);

    // start pathtracing!!! divvy up each row of pixels to different threads.
    for (int i = 0; i < samples; ++i) {
      parallel_for(
          0, OUTH,
          [&](int py) {
            for (int px = 0; px < OUTW; ++px) {
              vec dir = point_towards(px, py);
              pixels[py][px] += (i * pixels[py][px] + pathtrace(scene, origin, dir) * 255) / (i+1);
            }
            // p.increment();
          },
          threadcnt);

      if (cb != nullptr) cb();
    }
    std::println("done");
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
