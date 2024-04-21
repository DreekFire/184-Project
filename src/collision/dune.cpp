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
    m_meshDrawing.collide(pm); // Call the collide function of MeshDrawing
}

void Dune::render(GLShader& shader) {
    // Render the dune mesh using MeshDrawing
    m_meshDrawing.drawMesh(shader, point, 1.0); // Adjust scale as needed
}
