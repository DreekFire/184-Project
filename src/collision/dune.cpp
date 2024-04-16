#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "dune.h"
#include "../misc/mesh_drawing.h" // Include the MeshDrawing header

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Dune::collide(PointMass& pm) {
    float tangent_point = dot(pm.position - point, normal);
    if (tangent_point <= 0) {
        Vector3D correction_vector = -(tangent_point + SURFACE_OFFSET) * normal - pm.last_position + pm.position;
        correction_vector = correction_vector * (1 - friction);
        pm.position = pm.last_position - correction_vector;
    }
}

void Dune::render(GLShader& shader) {
    // Render the dune mesh using MeshDrawing
    m_meshDrawing.drawMesh(shader, point, 1.0); // Adjust scale as needed
}
