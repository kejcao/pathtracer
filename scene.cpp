module;

#include <memory>
#include <vector>

export module scene;

import object;
import math;

export class Scene {
public:
  BVH objects;

  Scene(const std::vector<Object *> &objects) : objects{objects} {}
  ~Scene() {}

  HitData castray(vec origin, vec direction) const {
    return objects.intersect(origin, direction);
  }

  std::vector<Object *> lights() const {
    std::vector<Object *> res;
    for (const auto &o : objects.objects) {
      if (o->mat->emission != vec(0, 0, 0)) {
        res.push_back(o);
      }
    }
    return res;
  }

private:
};
