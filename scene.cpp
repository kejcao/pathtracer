module;

#include <memory>
#include <vector>

export module scene;

import object;
import math;

export class Scene {
public:
    BVH objects;

    Scene(const std::vector<Object *> &objects) : objects{objects} { }
    ~Scene() { }

    HitData castray(vec origin, vec direction) const {
        return objects.intersect(origin, direction);
    }
private:
};
