#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include "lights.h"
#include "objects.h"

class Scene {
public:
    BVH objects;
    std::vector<PointLight *> pointlights;
    std::vector<AreaLight *> arealights;

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
    }

    ~Scene() {
        for (auto &&light : pointlights) delete light;
        for (auto &&light : arealights) delete light;
    }

    // AreaLight *random_light() const {
    //     return arealights[rand() % arealights.size()];
    // }

    HitData castray(vec origin, vec direction) const {
        return objects.intersect(origin, direction);
    }
private:
};

#endif