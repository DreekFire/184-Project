#include "parabolaBead.h";

#include "parabolaBead.h"

std::vector<std::vector<CGL::Vector3D>> ParabolaBead::resolve(std::vector<float> q) {
    std::vector<CGL::Vector3D> position = { CGL::Vector3D(q[0], q[0] * q[0], 0), };
    std::vector<CGL::Vector3D> gradient = { CGL::Vector3D(1, 2 * q[0], 0), };
    return {position, gradient}
}