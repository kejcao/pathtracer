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

enum {BVH, FLAT, PATHTRACE, RAYTRACE};

template<int CW, int CH, int MODE=RAYTRACE>
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

    Camera(const vec &origin, const vec &direction)
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

            // scalar theta = 2 * M_PI * randreal(0, 1);
            // scalar phi = std::acos(2 * randreal(0, 1) - 1);
            // return vec(
            //     std::sin(phi) * std::cos(theta),
            //     std::sin(phi) * std::sin(theta), std::cos(phi)
            // );
        };

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
            return
                mat->emission + 2 * M_PI * brdf *
                ( // the irradiance
                    std::abs(direction.normalize().dot(v)) *
                    pathtrace(scene, intersection, v, depth+1)
                ) / probability;
        }
        return vec(0, 0, 0);
    }

    vec point_towards(int px, int py) {
        return vec(
            (px - (scalar)CW/2) *  vw/CW,
            // We negate this so positive Y goes up and negative Y down in camera origin.
            (py - (scalar)CH/2) * -vh/CH,
            // See https://en.wikipedia.org/wiki/Field_of_view#Photography
            1/(2*std::tan(fov/2 * (M_PI/180))/vw)
        ).rotate(direction);
    }

    void render_pixel(const Scene &scene, int px, int py) {
        vec color = vec(0, 0, 0);

        const int samples = 128;
        for (int i = 0; i < samples; ++i) {
            color += pathtrace(scene, origin, direction);
        }
        color /= samples;
    }

    void render(const Scene &scene, const std::string &filename) {
        START();

        const int samples = 128;

        for (int i = 0; i < samples; ++i) {
            parallel_for(0, CH, [&](int py) {
                for (int px = 0; px < CW; ++px) {
                    vec dir = point_towards(px, py);
                    // render_pixel(scene, px, py);
                    pixels[py][px] += pathtrace(scene, origin, dir) * 10;
                }
            }, threadcnt);

            std::cout << "\r" << i << "/" << samples;
            std::cout.flush();
        }
        std::cout << "\r";

        for (int i = 0; i < CH; ++i) {
            for (int j = 0; j < CW; ++j) {
                pixels[CH][CW] /= samples;
            }
        }


        // frame /= samples;
        END("raytrace");

        std::ofstream of(filename);
        if (!of) assert(false);
        of << "P3\n" << CW << " " << CH << " 255" << "\n";
        for (const auto &row : pixels) {
            for (const auto &px : row) {
                of << std::clamp((int)px.x, 0, 255) << " "
                   << std::clamp((int)px.y, 0, 255) << " "
                   << std::clamp((int)px.z, 0, 255) << "\n";
                // of << (color>>16 & 0xff) << " " << (color>>8 & 0xff) << " " << (color & 0xff) << "\n";
            }
        }
    }

private:
    int vw = 1, vh = 1;
    vec origin, direction;
    vec pixels[CH][CW];

    // RGB components should be between 0 and 1, not 0–255.
    // void setpixel(int px, int py, vec rgb) {
    //     rgb = (rgb*255).floor().clamp(0, 255);
    //     pixels[py][px] = (int)rgb.x<<16 | (int)rgb.y<<8 | (int)rgb.z;
    // }
};

#endif