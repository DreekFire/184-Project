#include <iostream>

#include "linkageUtils.h"

std::pair<CGL::Vector2D, std::vector<CGL::Vector2D>> intersectPointDistances(CGL::Vector2D p, std::vector<CGL::Vector2D> dp, float r1, float r2, int direction) {
    // x^2 - (d-x)^2 = r1^2 - r2^2
    // 2x = r1^2 - r2^2 + d^2
    float a = (r1 * r1 - r2 * r2 + p.norm2()) / (2 * p.norm());
    // float a = (p.norm2() - (r1 * r1 - r2 * r2)) / (2 * p.norm());
    float b = sqrtf(r1 * r1 - a * a);
    CGL::Vector2D pUnit = p.unit();
    CGL::Vector2D normal(-pUnit.y, pUnit.x);
    CGL::Vector2D s = a * pUnit + b * normal * direction;

    // std::cout << "a: " << a << " b: " << b << " pUnit: " << pUnit << std::endl;
    // float daa = (r1 * r1 - r2 * r2) / (2 * a * a) + 0.5;
    // float dba = -a / b * daa;
    // float dab = -b;
    // float dbb = a;
    std::vector<CGL::Vector2D> vels(dp.size());
    CGL::Vector2D legUnit = (s - p).unit();
    CGL::Vector2D legNorm(-legUnit.y, legUnit.x);
    float legLever = dot(p, legNorm);
    for (auto v : dp) {
        float w = -dot(legUnit, v) * r1 / legLever;
        vels.push_back(CGL::Vector2D(-w * s.y, w * s.x));
        // float da = dot(v, p.unit());
        // float db = dot(v, binormal);
        // vels.push_back((daa * da + dab * db) * p + (dba * da + dbb * db) * binormal);
    }
    return std::make_pair(s, vels);
}