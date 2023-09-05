#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include "lights.h"
#include "objects.h"

class Scene {
public:
    std::vector<Object *> objects;
    std::vector<PointLight *> pointlights;
    std::vector<AreaLight *> arealights;
    BVH bvh;

    Scene(
        const std::vector<Object *> &objects,
        const std::vector<LightSource *> &lights
    ) : objects{objects} {
        for (auto *l : lights) {
            if (dynamic_cast<PointLight *>(l) != nullptr) {
                pointlights.push_back((PointLight *)l);
            } else if (dynamic_cast<AreaLight *>(l) != nullptr) {
                arealights.push_back((AreaLight *)l);
            }
        }
        bvh = BVH(this->objects);
    }

    ~Scene() {
        for (auto &&obj : objects) delete obj;
        for (auto &&light : pointlights) delete light;
        for (auto &&light : arealights) delete light;
    }

    HitData castray(vec origin, vec direction) const {
        return bvh.intersect(origin, direction);
        HitData closest = {INF};
        for (const auto &obj : objects) {
            HitData hit = obj->intersect(origin, direction);
            if (hit.t >= 1e-8 && hit.t < closest.t) {
                closest = hit;
            }
        }
        return closest;
    }
private:
};

#endif