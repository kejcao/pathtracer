#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>
#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "lights.h"
#include "math.h"
#include "objects.h"
#include "scene.h"
#include "util.h"

template<int OUTW, int OUTH>
class Camera {
public:
    scalar fov = 65;
    vec bgcolor = vec(0, 0, 0);

    // Simulate depth of field (blurry background) effects?
    bool DOF = true;

    // If so define these variables.
    scalar aperature = 2;
    scalar focal_distance = 4;
    bool stratified = false;
    int dof_samples = 512;

    // int arealight_samples = DOF ? 512 / dof_samples : 512;
    int arealight_samples = 1;
    int threadcnt = std::thread::hardware_concurrency();

    vec background_color = vec(0, 0, 0);
    vec ambient_light = vec(.1, .1, .1);

    Camera(const vec &origin, const vec &direction=vec(0,0,0))
        : origin{origin}, direction{direction} { }

    vec pathtrace(const Scene &scene, vec origin, vec direction, int depth=0) {
        HitData hit = scene.castray(origin, direction);
        if (hit.t == INF) return bgcolor; // nothing hit?

        // return random direction (unit sphere)
        auto random_direction = []() {
            // rejection sampling is slower.
            scalar x = randreal(0, 1);
            scalar y = randreal(0, 1);
            scalar phi = 1 - 2*x;
            scalar theta = std::fmax(0, 1 - phi*phi);
            return vec(
                cos(2*M_PI*y) * sqrt(theta),
                sin(2*M_PI*y) * sqrt(theta), phi
            ).normalize();
        };

        vec color = vec(0,0,0);

        // russian roulette!!!
        const double probability = .9;
        if (depth <= 4 || randreal(0,1) < probability) {
            auto mat = hit.obj->mat;
            vec brdf = mat->diffuse / M_PI;
            vec v = random_direction();

            // if not in hemisphere around normal, flip it.
            // TODO normal should be to the outside of mesh.
            if (hit.normal.dot(v) > 0) v = -v;

            vec intersection = origin + direction*hit.t;
            // corresponds to rendering equation.
            color +=
                mat->emission + 2 * M_PI * brdf *
                ( // the irradiance
                    std::abs(direction.normalize().dot(v)) *
                    pathtrace(scene, intersection, v, depth+1)
                ) / probability;
        }
        return color;
    }

    vec point_towards(int px, int py) {
        return vec(
            (px - (scalar)OUTW/2) *  vw/OUTW,
            // We negate this so positive Y goes up and negative Y down in camera origin.
            (py - (scalar)OUTH/2) * -vh/OUTH,
            // See https://en.wikipedia.org/wiki/Field_of_view#Photography
            1/(2*std::tan(fov/2 * (M_PI/180))/vw)
        ).rotate(direction);
    }

    void render(
        const Scene &scene,
        const std::string &outfp,
        int samples = 64
    ) {
        START();

        for (int i = 0; i < samples; ++i) {
            parallel_for(0, OUTH, [&](int py) {
                for (int px = 0; px < OUTW; ++px) {
                    vec dir = point_towards(px, py);
                    pixels[py][px] += pathtrace(scene, origin, dir) * 255;
                }
            }, threadcnt);

            std::cout << "\r" << i << "/" << samples;
            std::cout.flush();
        }
        std::cout << "\r";

        for (auto &row : pixels) {
            for (auto &px : row) {
                px /= samples;
            }
        }

        END("raytrace");

        std::ofstream of(outfp);
        if (!of) assert(false);
        of << "P3\n" << OUTW << " " << OUTH << " 255" << "\n";
        for (const auto &row : pixels) {
            for (const auto &px : row) {
                of << std::clamp((int)std::round(px.x), 0, 255) << " "
                   << std::clamp((int)std::round(px.y), 0, 255) << " "
                   << std::clamp((int)std::round(px.z), 0, 255) << "\n";
            }
        }
    }

private:
    int vw = 1, vh = 1;
    vec origin, direction;
    vec pixels[OUTH][OUTW];
};

#endif