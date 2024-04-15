#include "jansen.h";

#include "CGL/vector2D.h"
#include "linkageUtils.h"

const std::vector<CGL::Vector2D> Jansen::fixedPos = {
    CGL::Vector2D(0, 0),
    CGL::Vector2D(38, 7.82),
};

const std::vector<std::vector<float>> Jansen::linkLengths = {
    {41.5f, 50.0f},
    {40.1f, 55.8f},
    {39.3f, 61.9f},
    {39.4f, 36.7f},
    {49.0f, 65.7f},
};

const std::vector<std::vector<int>> Jansen::resolutionOrder = {
    {3, 0, 2, 1},
    {4, 0, 3, 1},
    {6, 0, 2, -1},
    {5, 6, 4, -1},
    {7, 6, 5, -1},
};

std::vector<std::vector<CGL::Vector2D>> Jansen::resolve(std::vector<float> q) {
    std::vector<CGL::Vector2D> position(8);
    position.insert(position.begin(), fixedPos.begin(), fixedPos.end());
    position[2] = fixedPos[1] + CGL::Vector2D(15 * cosf(q[0]), 15 * sinf(q[0]));
    std::vector<std::vector<CGL::Vector2D>> velocity(8);
    velocity[0] = velocity[1] = { CGL::Vector2D(0, 0), };
    velocity[2] = { CGL::Vector2D(-15 * sinf(q[0]), 15 * cosf(q[0])), };
    for (int i = 0; i < resolutionOrder.size(); i++) {
        int idx1 = resolutionOrder[i][1];
        int idx2 = resolutionOrder[i][1];
        std::pair<CGL::Vector2D, std::vector<CGL::Vector2D>> p = intersectPointDistances(
            position[idx2] - position[idx1],
            {velocity[idx2][0] - velocity[idx1][0]},
            linkLengths[i][0], linkLengths[i][1],
            resolutionOrder[i][3]
        );
    }
    return {position, velocity};
}