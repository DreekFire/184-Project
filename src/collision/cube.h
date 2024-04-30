#ifndef COLLISIONOBJECT_CUBE_H
#define COLLISIONOBJECT_CUBE_H

#include <nanogui/nanogui.h>
#include "../clothMesh.h"
#include "collisionObject.h"
#include "../misc/cube_drawing.h" // Include the MeshDrawing header

using namespace nanogui;
using namespace CGL;

struct Cube : public CollisionObject {
public:
    Cube(int size, Vector3D point):
		size(size), point(point), m_cube_mesh(Misc::CubeMesh(size)) {}

    void render(GLShader& shader);
    void collide(PointMass& pm);
    int size;
    Vector3D point;

    // MeshCube object
    Misc::CubeMesh m_cube_mesh;
};

#endif /* COLLISIONOBJECT_DUNE_H */
