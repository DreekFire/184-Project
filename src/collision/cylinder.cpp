#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/cylinder_drawing.h"
#include "cylinder.h"

using namespace nanogui;
using namespace CGL;

void Cylinder::collide(PointMass &pm) {
  Vector3D direction = pm.position - origin;
  Vector3D closest_point_on_axis = origin + (dot(direction, axis) / axis.norm2()) * axis;
  
  Vector3D to_closest_point = closest_point_on_axis - pm.position;
  
  if (to_closest_point.norm() < radius) {
    Vector3D tangent = direction - to_closest_point;
    Vector3D correction_vector = (tangent.unit() * radius) - to_closest_point;
    
    pm.position = pm.last_position + correction_vector * (1 - friction);
  }
}

void Cylinder::render(GLShader &shader) {
  m_cylinder_mesh.draw_cylinder(shader, origin, axis, radius * 0.92, height);
}
