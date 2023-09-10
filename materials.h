#ifndef MATERIAL_H
#define MATERIAL_H

#include "math.h"

class Material {
public:
    vec diffuse = vec(.5, .5, .5);
    vec specular = vec(1, 1, 1);
    vec emission = vec(0, 0, 0);
    scalar shininess = 20;
    scalar index_of_refraction = 1;

    Material() = default;
    Material(vec diffuse, vec specular, scalar shininess)
    : diffuse{diffuse}, specular{specular}, shininess{shininess} {}
};

#endif