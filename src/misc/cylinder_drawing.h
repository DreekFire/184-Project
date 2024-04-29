#ifndef CGL_UTIL_CYLINDERDRAWING_H
#define CGL_UTIL_CYLINDERDRAWING_H

#include <vector>

#include <nanogui/nanogui.h>

#include "CGL/CGL.h"

using namespace nanogui;

namespace CGL {
namespace Misc {
  
class CylinderMesh {
public:
  // Supply the desired number of slices and stacks
  CylinderMesh(int num_slices = 40, int num_stacks = 40);
  
  /**
   * Draws a cylinder with the given position, radius, and height in opengl, using the
   * current modelview/projection matrices and color/material settings.
   */
  void draw_cylinder(GLShader &shader, const Vector3D &axis, const Vector3D &p, double r, double h);
  
private:
  std::vector<unsigned int> Indices;
  std::vector<double> Vertices;
  
  int c_index(int stack, int slice);
  
  void build_data();
  
  int cylinder_num_slices;
  int cylinder_num_stacks;
  
  int cylinder_num_vertices;
  int cylinder_num_indices;
  
  MatrixXf positions;
  MatrixXf normals;
  MatrixXf uvs;
  MatrixXf tangents;
};

} // namespace Misc
} // namespace CGL

#endif // CGL_UTIL_CYLINDERDRAWING_H