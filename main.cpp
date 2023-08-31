#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <ranges>
#include <thread>
#include <vector>
#include <ranges>
#include <random>

// constexpr double INF = std::numeric_limits<double>::infinity();
#define INF std::numeric_limits<double>::infinity()

namespace config {
    constexpr bool multithread = true;
    constexpr double ambient_light = 40;
    constexpr int area_light_samples = 64;

    const unsigned int threadcnt = std::thread::hardware_concurrency();
}

class vec {
public:
    double x, y, z;

    vec() = default;
    vec(double x, double y, double z) : x{x}, y{y}, z{z} { }
    vec(const vec &v) : x{v.x}, y{v.y}, z{v.z} { };

    vec operator+(const vec &v) const { return vec(x + v.x, y + v.y, z + v.z); }
    vec operator-(const vec &v) const { return vec(x - v.x, y - v.y, z - v.z); }
    vec operator*(const vec &v) const { return vec(x * v.x, y * v.y, z * v.z); }
    vec operator/(const vec &v) const { return vec(x / v.x, y / v.y, z / v.z); }
    void operator+=(const vec &v) { x += v.x; y += v.y; z += v.z; }
    void operator-=(const vec &v) { x -= v.x; y -= v.y; z -= v.z; }
    void operator*=(const vec &v) { x *= v.x; y *= v.y; z *= v.z; }
    void operator/=(const vec &v) { x /= v.x; y /= v.y; z /= v.z; }
    bool operator==(const vec &) const = default;
    bool operator!=(const vec &) const = default;
    double dot(const vec &v) const { return x*v.x + y*v.y + z*v.z; }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }

    vec normalize() const {
        double n = norm();
        return n == 0 ? vec(x,y,z) : vec(x/n, y/n, z/n);
    }

    vec cross(const vec &v) const {
        return vec(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.y - y*v.x
        );
    }

    // Should totally precompute the rotation matrix. Maybe make transformation class? TODO
    vec rotate(double a, double b, double y) const {
        return vec(
            (cos(a)*cos(y) * this->x) + (sin(a)*sin(b)*cos(y) - cos(a)*sin(y) * this->y) + (cos(a)*sin(b)*cos(y) + sin(a)*sin(y) * this->z),
            (cos(a)*sin(y) * this->x) + (sin(a)*sin(b)*sin(y) + cos(a)*cos(y) * this->y) + (cos(a)*sin(b)*sin(y) - sin(a)*cos(y) * this->z),
                  (-sin(b) * this->x) +                        (sin(a)*cos(b) * this->y) +                        (cos(a)*cos(b) * this->z)
        );
    }

    vec reflect_around(const vec &normal) const;
};

vec operator*(double n, const vec &v) { return vec(v.x * n, v.y * n, v.z * n); }
vec operator*(const vec &v, double n) { return vec(v.x * n, v.y * n, v.z * n); }

vec vec::reflect_around(const vec &normal) const {
    return 2*this->dot(normal)*normal - *this;
}

std::ostream &operator<<(std::ostream &os, const vec &v) {
    return os << "[" << v.x << "," << v.y << "," << v.z << "]\n";
}

struct HitData {
    double t;
    vec normal;
};

class Object {
public:
    virtual ~Object() = default;
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
        bool smooth
    ) : v0{v0}, v1{v1}, v2{v2},
        vn0{vn0}, vn1{vn1}, vn2{vn2},
        smooth{smooth} {
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
        return {q.dot(e2), smooth ? (1-u-v)*vn0 + u*vn1 + v*vn2 : normal};
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
                v3[1] < vnormals.size() ? vnormals[v3[1]] : vertices[v3[0]], smooth
            ));
        };

        for (const auto &face : faces) {
            if (face.size() > 3) {
                std::array<unsigned int, 2> pivot = face[0];
                for (size_t i = 1; i < face.size()-1; ++i) {
                    this->faces.push_back({pivot, face[i], face[i+1]});
                    addtriangle(pivot, face[i], face[i+1]);
                    // triangles.push_back(Triangle(
                    //     vertices[pivot[0]], vertices[face[i][0]], vertices[face[i+1][0]],
                    //     vnormals[pivot[1]], vnormals[face[i][1]], vnormals[face[i+1][1]], smooth
                    // ));
                }
            } else {
                this->faces.push_back({face[0], face[1], face[2]});
                addtriangle(face[0], face[1], face[2]);
                // triangles.push_back(Triangle(
                //     vertices[face[0][0]], vertices[face[1][0]], vertices[face[2][0]],
                //     vnormals[face[0][1]], vnormals[face[1][1]], vnormals[face[2][1]], smooth
                // ));
            }
        }
    }

    HitData intersect(vec origin, vec direction) const override {
        HitData closest = {INF};
        for (const auto &triangle : triangles) {
            HitData hit = triangle.intersect(origin, direction);
            if (hit.t < closest.t) {
                closest = hit;
            }
        }
        return closest;
    }
private:
    std::vector<std::array<std::array<unsigned int, 2>, 3>> faces;
    std::vector<Triangle> triangles;
};

class LightSource {
public:
    vec origin;
    LightSource(const vec &origin) : origin{origin} { }
    // virtual double compute_illumination(vec source, vec intersection, vec normal) const = 0;
};

class PointLight : public LightSource {
public:
    using LightSource::LightSource;
    // double compute_illumination(vec source, vec intersection, vec normal) const override {
    //     vec destination = (origin - intersection).normalize();
    //     vec reflection = (destination.reflect_around(normal)).normalize();
    //     return 180 * std::max(0.0, destination.dot(normal))
    //          + 255 * pow(std::max(0.0, reflection.dot(source)), 20);

    //     // If there is no object between the intersection point and the
    //     // light source or if the intersected object is behind the light
    //     // source, then calculate light. Otherwise there is shadow.
    //     // double distance = castray(scene, intersection, destination).t;
    //     // if (distance == INF || distance > (light->origin - intersection).norm()) {
    //         // Don't make this mistake!!! pow(negative number, 20) = positive number.
    //         // total += 255 * std::max(0.0, pow(reflection.dot(source), 20));
    //     // }
    // }

    // double compute_illumination(vec intersection) const override {
    //     vec destination = (origin - intersection).normalize();
    //     // If there is no object between the intersection point and the
    //     // light source or if the intersected object is behind the light
    //     // source, then calculate light. Otherwise there is shadow.
    //     double distance = castray(scene, intersection, destination).t;
    //     if (distance == INF || distance > (light->origin - intersection).norm()) {
    //         vec reflection = (destination.reflect_around(normal)).normalize();
    //         total += 180 * std::max(0.0, destination.dot(normal));
    //         total += 255 * pow(std::max(0.0, reflection.dot(source)), 20);
    //         // Don't make this mistake!!! pow(negative number, 20) = positive number.
    //         // total += 255 * std::max(0.0, pow(reflection.dot(source), 20));
    //     }
    // }
};

class AreaLight : public LightSource {
public:
    vec direction;
    int radius = 1;
    AreaLight(const vec &origin, const vec &direction) : LightSource(origin), direction{direction} {}
    using LightSource::LightSource;
private:
};

class Scene {
public:
    std::vector<Object *> objects;
    std::vector<LightSource *> lights;
    std::vector<AreaLight *> arealights;

    Scene(
        const std::vector<Object *> &objects,
        const std::vector<LightSource *> &lights,
        const std::vector<AreaLight *> &arealights
    ) : objects{objects}, lights{lights}, arealights{arealights} { }

    ~Scene() {
        for (auto &&obj : objects) delete obj;
        for (auto &&light : lights) delete light;
        for (auto &&light : arealights) delete light;
    }
private:
};

template<int CW, int CH>
class Camera {
public:
    Camera(const vec &origin, const vec &direction) : origin{origin}, direction{direction} { }

    HitData castray(const Scene &scene, vec origin, vec direction) {
        HitData closest = {INF};
        for (const auto &obj : scene.objects) {
            HitData hit = obj->intersect(origin, direction);
            if (hit.t >= 1e-10 && hit.t < closest.t) {
                closest = hit;
            }
        }
        return closest;
    }

    void render_pixel(const Scene &scene, int px, int py) {
        vec direction = vec(
            (px - (double)CW/2) *  vw/CW,
            // We negate this so positive Y goes up and negative y down in camera origin.
            (py - (double)CH/2) * -vh/CH, 1
        ).rotate(this->direction.x, this->direction.y, this->direction.z);

        HitData closest = castray(scene, origin, direction);
        if (closest.t == INF) {
            setpixel(px,py, 0,0,0);
            return;
        }

        vec intersection = origin + direction*closest.t;
        vec normal = closest.normal.normalize();
        vec source = (origin - intersection).normalize();

        auto compute_illumination = [&](vec light_origin) {
            vec destination = (light_origin - intersection).normalize();
            // If there is no object between the intersection point and the
            // light source or if the intersected object is behind the light
            // source, then calculate light. Otherwise there is shadow.
            double distance = castray(scene, intersection, destination).t;
            if (distance == INF || distance > (light_origin - intersection).norm()) {
                vec reflection = (destination.reflect_around(normal)).normalize();
                return 180 * std::max(0.0, destination.dot(normal)) +
                       255 * pow(std::max(0.0, reflection.dot(source)), 20);
                // Don't make this mistake!!! pow(negative number, 20) = positive number.
                // total += 255 * std::max(0.0, pow(reflection.dot(source), 20));
            }
            return 0.0;
        };

        double total = config::ambient_light;
        for (const auto &light : scene.lights) {
            total += compute_illumination(light->origin);
        }

        std::random_device rd;
        std::mt19937 e2(rd());
        for (const auto &light : scene.arealights) {
            for (int i = 0; i < config::area_light_samples; ++i) {
                double x, y;
                do {
                    x = std::uniform_real_distribution<>(-light->radius, light->radius)(e2);
                    y = std::uniform_real_distribution<>(-light->radius, light->radius)(e2);
                } while(sqrt(x*x + y*y) > light->radius);
                total += compute_illumination(light->origin + vec(x, y, 0)) / config::area_light_samples;
            }
        }
        total = std::min(std::max((int)total, 0), 255);
        setpixel(px,py, total,total,total);
    }

    void render(const Scene &scene, int py) {
        for (int px = 0; px < CW; ++px) {
            render_pixel(scene, px, py);
        }
    }

    void render(const Scene &scene, const std::string &filename) {
        auto start = std::chrono::high_resolution_clock::now();

        if (config::multithread) {
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
            for (auto i = 0; i < config::threadcnt-1; ++i)
                threads.push_back(std::thread(runner));
            runner();
            for (auto &&t : threads) t.join();
            std::cout << "\r";
        } else {
            // TODO progress bar doesn't work :(
            for (int py = 0; py < CH; ++py) {
                render(scene, py);
                std::cout << "\r" << py*100 / CH << "%";
                std::cout.flush();
            }
            std::cout << "\r";
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << std::fixed << std::setprecision(2) << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / (double)1000 << " seconds elapsed\n";

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

    void setpixel(int px, int py, int r, int g, int b) {
        pixels[py][px] = r<<16 | g<<8 | b;
    }
};

std::vector<Object *> parse(const std::string &filename) {
    auto split = [](
        const std::string &s,
        const std::string &delim=" "
    ) {
        std::vector<std::string> toks = {};
        size_t i = 0, pi = 0;
        while ((i = s.find(delim, pi)) != std::string::npos) {
            // prune empty strings
            if (i-pi != 0) {
                toks.push_back(s.substr(pi, i-pi));
            }
            // toks.push_back(s.substr(pi, i-pi));
            pi = i + delim.size();
        }
        toks.push_back(s.substr(pi));
        return toks;
    };

    std::ifstream ifs(filename);
    if (!ifs) assert(false);

    std::vector<Object *> objects;
    int offset = 0;
    Polygon *p = nullptr;
    std::string line;
    while (std::getline(ifs, line)) {
        std::vector<std::string> toks = split(line);

        if (toks[0] == "o") {
            if (p != nullptr) {
                objects.push_back(p);
                offset += p->vertices.size();
            }
            p = new Polygon();
            continue;
        } else if (toks[0] == "s") {
            assert(toks.size() == 2);
            p->smooth = std::stoi(toks[1]) == 1;
        } else if (toks[0] == "v") {
            assert(toks.size() == 4);
            p->vertices.push_back(vec(
                std::stof(toks[1]),
                std::stof(toks[2]),
                std::stof(toks[3])
            ));
        } else if (toks[0] == "vn") {
            assert(toks.size() == 4);
            p->vnormals.push_back(vec(
                std::stof(toks[1]),
                std::stof(toks[2]),
                std::stof(toks[3])
            ));
        } else if(toks[0] == "f") {
            std::vector<std::array<unsigned int, 2>> vs;
            for (const auto &vertex : toks | std::views::drop(1)) {
                std::vector<std::string> toks = split(vertex, "/");
                if (toks.size() == 1) {
                    vs.push_back({ (unsigned int)std::stoi(toks[0])-1 - offset, 0 });
                } else {
                    assert(toks.size() == 3);
                    vs.push_back({
                        (unsigned int)std::stoi(toks[0])-1 - offset,
                        (unsigned int)std::stoi(toks[2])-1 - offset
                    });
                }
            }
            p->addfaces({vs});
        }
    }
    if (p != nullptr) {
        objects.push_back(p);
    }
    return objects;
}

int main(void) {
    // Camera<256, 256>(vec(0, 0, -3), vec(0, 0, 0)).render(
    //     Scene(parse("sphere.obj"), {new PointLight(vec(0, 5, 0))}), "out.ppm");
    // Camera<256, 256>(vec(0, 5, -10), vec(.3, 0, 0)).render(
    //     Scene(parse("cube.obj"), {new PointLight(vec(0, 5, -2))}, {}), "out.ppm");
    Camera<256, 256>(vec(0, 5, -10), vec(.3, 0, 0)).render(
        Scene(parse("examples/cube.obj"), {}, {new AreaLight(vec(0, 5, -2), vec(0, 0, 0))}), "out.ppm");
    // Camera<1024, 1024>(vec(0, 0, 10), vec(0, 0, 0)).render(
    //     Scene(parse("teapot.obj"), {
    //         new PointLight(vec(0, 5, 0)),
    //         new PointLight(vec(0, -5, 5))
    //     }), "out.ppm");
    // Camera<256, 256>(vec(0, 1, 10), vec(0, 0, 0)).render(
    //     Scene(parse("teapot.obj"), {new PointLight(vec(0, 5, 0))}), "out.ppm");
    // Camera<256, 256>(vec(0, 1, 10), vec(-.2, 0, 0)).render(
    //     Scene(parse("scene.obj"), {new PointLight(vec(0, 30, 0))}), "out.ppm");
}