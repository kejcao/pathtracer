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

template<int CW, int CH>
class Camera {
public:
    scalar fov = 65;
    scalar aperature = 50;
    scalar focal_distance = 4;
    int threadcnt = std::thread::hardware_concurrency();
    vec background_color = vec(0, 0, 0);
    vec ambient_light = vec(.1, .1, .1);

    Camera(const vec &origin, const vec &direction) : origin{origin}, direction{direction} { }

    vec pathtrace(const Scene &scene, vec origin, vec direction, int depth=4) {
        if (depth <= 0) {
            return vec(0, 0, 0);
        }
        HitData hit = scene.castray(origin, direction);
        if (hit.t == INF) {
            return vec(0, 0, 0);
        }

        auto mat = hit.obj->mat;

        // vec v;
        // do {
        //     v = vec(
        //         randreal(-1, 1),
        //         randreal(-1, 1),
        //         randreal(-1, 1)
        //     );
        // } while(std::hypot(v.x, v.y, v.z) > 1);
        // v = v.normalize();

        scalar x = randreal(0, 1);
        scalar y = randreal(0, 1);
        scalar phi = 1 - 2*x;
        scalar theta = std::fmax(0, 1 - phi*phi);
        vec v = vec(
            cos(2*M_PI*y) * sqrt(theta),
            sin(2*M_PI*y) * sqrt(theta), phi
        ).normalize();

        vec intersection = origin + direction*hit.t;
        vec normal = hit.normal.normalize();
        vec source = (origin - intersection).normalize();

        return 20*mat->emission + 2 * mat->diffuse * std::abs(direction.normalize().dot(v)) * pathtrace(scene, intersection, v, depth-1);
    }

    vec raytrace(const Scene &scene, vec origin, vec direction, int depth=1) {
        if (depth <= 0) {
            return vec(0, 0, 0);
        }
        HitData hit = scene.castray(origin, direction);
        if (hit.t == INF) {
            return background_color;
        }

        vec intersection = origin + direction*hit.t;
        vec normal = hit.normal.normalize();
        vec source = (origin - intersection).normalize();

        auto mat = hit.obj->mat;

        auto compute_illumination = [&](vec light_origin) -> vec {
            // If there is no object between the intersection point and the
            // light source or if the intersected object is behind the light
            // source, then calculate light. Otherwise there is shadow.
            vec destination = (light_origin - intersection).normalize();
            scalar distance = scene.castray(intersection, destination).t;
            if (distance == INF || distance > (light_origin - intersection).norm()) {
                vec reflection = (destination.reflect_around(normal)).normalize();
                return mat->diffuse * std::abs(destination.dot(normal)) +
                       mat->specular * pow(std::fmax(0.0, reflection.dot(source)), mat->shininess) +
                       .1 * raytrace(scene, intersection, reflection, depth-1);
            }
            return vec(0, 0, 0);
        };

        vec color = ambient_light;
        for (const auto &light : scene.pointlights) {
            color += compute_illumination(light->origin);
        }
        for (const auto &light : scene.arealights) {
            const int samples = 128;
            // const int samples = 1024;
            for (int i = 0; i < samples; ++i) {
                // Generate random point on circle facing the intersection point.
                scalar x, y;
                do {
                    x = randreal(-light->radius, light->radius);
                    y = randreal(-light->radius, light->radius);
                } while(std::hypot(x, y) > light->radius);
                color += compute_illumination(light->origin + vec(x, y, 0)) / samples;
            }
        }
        return color;
    }

    void render_pixel(const Scene &scene, int px, int py) {
        vec direction = vec(
            (px - (scalar)CW/2) *  vw/CW,
            // We negate this so positive Y goes up and negative Y down in camera origin.
            (py - (scalar)CH/2) * -vh/CH,
            // See https://en.wikipedia.org/wiki/Field_of_view#Photography
            1/(2*std::tan(fov/2 * (M_PI/180))/vw)
        ).rotate(this->direction.x, this->direction.y, this->direction.z);

        vec color = vec(0, 0, 0);
        enum {BVH, FLAT, PATHTRACE, RAYTRACE};

        int VIEW_MODE = PATHTRACE;
        switch (VIEW_MODE) {
        case BVH:
            color = vec((double)scene.castray(origin, direction).hits / 30);
            break;
        case FLAT: {
            HitData hit = scene.castray(origin, direction);
            if (hit.t != INF) color = hit.obj->mat->diffuse;
            break;
        }
        case PATHTRACE: {
            const int samples = 1024;
            for (int i = 0; i < samples; ++i) {
                color += pathtrace(scene, origin, direction);
            }
            color /= samples;
            break;
        }
        case RAYTRACE:
            const int samples = 256;
            vec focal_point = origin + direction*focal_distance;
            for (int i = 0; i < samples; ++i) {
                // Note each pixel has width vw/CW in viewport space.
                vec neworigin = origin + vec(
                    randreal(-(scalar)vw/CW / 2, (scalar)vw/CW / 2) * aperature,
                    randreal(-(scalar)vw/CH / 2, (scalar)vw/CH / 2) * aperature, 0
                );
                color += raytrace(scene, neworigin, focal_point - neworigin) / samples;
            }
            break;
        }
        setpixel(px,py, color);
    }

    void render(const Scene &scene, const std::string &filename) {
        START();
        parallel_for(0, CH, [&](int py) {
            for (int px = 0; px < CW; ++px) {
                render_pixel(scene, px, py);
            }
        }, threadcnt);
        END("raytrace");

        std::ofstream of(filename);
        if (!of) assert(false);
        of << "P3\n" << CW << " " << CH << " 255" << "\n";
        for (auto &&row : pixels) {
            for (auto &&color : row) {
                of << (color>>16 & 0xff) << " " << (color>>8 & 0xff) << " " << (color & 0xff) << "\n";
            }
        }
    }

private:
    int vw = 1, vh = 1;
    vec origin, direction;
    int pixels[CH][CW];

    // RGB components should be between 0 and 1, not 0–255.
    void setpixel(int px, int py, vec rgb) {
        rgb = (rgb*255).floor().clamp(0, 255);
        pixels[py][px] = (int)rgb.x<<16 | (int)rgb.y<<8 | (int)rgb.z;
    }
};

#endif