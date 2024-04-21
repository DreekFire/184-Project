#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "cube.h"
#include "../misc/cube_drawing.h" // Include the CubeDrawing header

using namespace std;
using namespace CGL;

void Cube::render(GLShader& shader) {
    // Render the dune mesh using MeshDrawing
    m_cube_mesh.draw_cube(shader, point, size);
}

// fake collision detection
void Cube::collide(PointMass& pm) {
	return ;
}
