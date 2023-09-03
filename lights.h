#ifndef LIGHTS_H
#define LIGHTS_H

#include "math.h"

class LightSource {
public:
    vec origin;
    LightSource(const vec &origin) : origin{origin} { }
    virtual ~LightSource() = default;
};

class PointLight : public LightSource {
public:
    using LightSource::LightSource;
};

class AreaLight : public LightSource {
public:
    vec direction;
    int radius = 1;
    AreaLight(const vec &origin, const vec &direction) : LightSource(origin), direction{direction} {}
    using LightSource::LightSource;
private:
};

#endif