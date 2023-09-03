#ifndef MATERIAL_H
#define MATERIAL_H

#include "math.h"

class Material {
public:
    vec diffuse = vec(.5, .5, .5);
    vec specular = vec(1, 1, 1);
    double shininess = 20;
    double index_of_refraction = 1;

    Material() = default;
    Material(vec diffuse, vec specular, double shininess)
    : diffuse{diffuse}, specular{specular}, shininess{shininess} {}
};

#endif