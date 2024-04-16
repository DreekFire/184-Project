#ifndef COLLISIONOBJECT_DUNE_H
#define COLLISIONOBJECT_DUNE_H

#include <nanogui/nanogui.h>
#include "../clothMesh.h"
#include "collisionObject.h"
#include "../misc/mesh_drawing.h" // Include the MeshDrawing header

using namespace nanogui;
using namespace CGL;

#define PATH "/textures/dune.jpg"

struct Dune : public CollisionObject {
public:
    Dune(const Vector3D& point, const Vector3D& normal, double friction, string project_root)
        : point(point), normal(normal.unit()), friction(friction), m_meshDrawing(Misc::MeshDrawing(PATH, project_root)) {} // Initialize MeshDrawing with texture path

    void render(GLShader& shader);
    void collide(PointMass& pm);

    Vector3D point;
    Vector3D normal;
    double friction;

    // MeshDrawing object for rendering the dune mesh
    Misc::MeshDrawing m_meshDrawing;
};

#endif /* COLLISIONOBJECT_DUNE_H */
