#ifndef CAMERA_H
#define CAMERA_H

#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "math.h"
#include "scene.h"

template<int CW, int CH>
class Camera {
public:
    double fov = 65;
    double aperature = 50;
    int threadcnt = std::thread::hardware_concurrency();
    vec background_color = vec(0, 0, 0);
    vec ambient_light = vec(.1, .1, .1);

    Camera(const vec &origin, const vec &direction) : origin{origin}, direction{direction} { }

    vec raytrace(const Scene &scene, vec origin, vec direction, int depth=4) {
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

        auto compute_illumination = [&](vec light_origin) -> vec {
            // If there is no object between the intersection point and the
            // light source or if the intersected object is behind the light
            // source, then calculate light. Otherwise there is shadow.
            vec destination = (light_origin - intersection).normalize();
            double distance = scene.castray(intersection, destination).t;
            if (distance == INF || distance > (light_origin - intersection).norm()) {
                vec reflection = (destination.reflect_around(normal)).normalize();
                auto mat = hit.obj->mat;
                return mat->diffuse * std::max(0.0, destination.dot(normal)) +
                       mat->specular * pow(std::max(0.0, reflection.dot(source)), 20) +
                       .2 * raytrace(scene, intersection, reflection, depth-1);
            }
            return vec(0, 0, 0);
        };

        vec color = ambient_light;
        for (const auto &light : scene.pointlights) {
            color += compute_illumination(light->origin);
        }
        for (const auto &light : scene.arealights) {
            for (int i = 0; i < 128; ++i) {
                double x, y;
                do {
                    x = randreal(-light->radius, light->radius);
                    y = randreal(-light->radius, light->radius);
                } while(sqrt(x*x + y*y) > light->radius);
                color += compute_illumination(light->origin + vec(x, y, 0)) / 128;
            }
        }

        return color;
    }

    void render_pixel(const Scene &scene, int px, int py) {
        const double focal_length = 1/(2*std::tan(fov/2 * (M_PI/180))/vw);

        vec direction = vec(
            (px - (double)CW/2) *  vw/CW,
            // We negate this so positive Y goes up and negative Y down in camera origin.
            (py - (double)CH/2) * -vh/CH, focal_length
        ).rotate(this->direction.x, this->direction.y, this->direction.z);

        setpixel(px,py, raytrace(scene, origin, direction));

        vec color = vec(0, 0, 0);
        if (true) {
            color = raytrace(scene, origin, direction);
        } else {
            for (int i = 0; i < 128; ++i) {
                vec neworigin = origin + vec(
                    randreal(-(double)vw/CW / 2, (double)vw/CW / 2) * aperature,
                    randreal(-(double)vw/CH / 2, (double)vw/CH / 2) * aperature, 0
                );
                vec focal_point = neworigin + direction*focal_length;
                color += raytrace(scene, neworigin, focal_point - neworigin) / 128;
            }
        }
        setpixel(px,py, color);
    }

    void render(const Scene &scene, int py) {
        for (int px = 0; px < CW; ++px) {
            render_pixel(scene, px, py);
        }
    }

    void render(const Scene &scene, const std::string &filename) {
        auto start = std::chrono::high_resolution_clock::now();
        if (threadcnt > 1) {
            int py = -1; // Start at -1 so the first value in `runner` is 0.
            auto runner = [&]{
                for (;;) {
                    int py_;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        if ((py_ = ++py) >= CH) return;
                        std::cout << "\r" << py*100 / CH << "%";
                        std::cout.flush();
                    }
                    for (int px = 0; px < CW; ++px) {
                        render_pixel(scene, px, py_);
                    }
                }
            };
            std::vector<std::thread> threads;
            // We spin up "threadcnt-1" threads because the main thread works too.
            for (int i = 0; i < threadcnt-1; ++i)
                threads.push_back(std::thread(runner));
            runner();
            for (auto &&t : threads) t.join();
            std::cout << "\r";
        } else {
            for (int py = 0; py < CH; ++py) {
                render(scene, py);
                std::cout << "\r" << py*100 / CH << "%";
                std::cout.flush();
            }
            std::cout << "\r";
        }
        auto end = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << std::fixed << std::setprecision(2) << ms/1000 << " seconds elapsed\n";

        std::ofstream of("out.ppm");
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
    std::mutex mutex;

    // RGB components should be between 0 and 1, not 0–255.
    void setpixel(int px, int py, vec rgb) {
        rgb = (rgb*255).floor().clamp(0, 255);
        pixels[py][px] = (int)rgb.x<<16 | (int)rgb.y<<8 | (int)rgb.z;
    }
};

#endif