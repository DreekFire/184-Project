#ifndef PARABOLA_INTERNAL_H
#define PARABOLA_INTERNAL_H

#include <vector>
#include <CGL/vector3D.h>

class ParabolaBead {
public:
    static std::vector<std::vector<CGL::Vector3D>> resolve(std::vector<float> q);
};

#endif