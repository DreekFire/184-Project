#ifndef LINKAGE_UTIL_H
#define LINKAGE_UTIL_H

#include <vector>
#include "CGL/vector2D.h"

std::pair<CGL::Vector2D, std::vector<CGL::Vector2D>> intersectPointDistances(CGL::Vector2D p, std::vector<CGL::Vector2D> dp, float r1, float r2, bool up);

#endif