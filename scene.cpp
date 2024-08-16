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

  // Object *light_source() const {
  //   // TOOD make this random
  //   for (const auto &o : objects) {
  //     if (o.mat->emission != vec(0, 0, 0)) {
  //       return o;
  //     }
  //   }
  // }

private:
};
