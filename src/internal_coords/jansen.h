#ifndef JANSEN_INTERNAL_H
#define JANSEN_INTERNAL_H

#include <vector>
#include <CGL/vector3D.h>

class Jansen {
public:
    static std::vector<std::vector<CGL::Vector2D>> resolve(std::vector<float> q);

private:
    const static std::vector<CGL::Vector2D> fixedPos;
    const static std::vector<std::vector<int>> resolutionOrder;
    const static std::vector<std::vector<float>> linkLengths;
};

#endif