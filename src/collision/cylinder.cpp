#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/cylinder_drawing.h"
#include "cylinder.h"

using namespace nanogui;
using namespace CGL;

void Cylinder::collide(PointMass &pm) {
  // Handle collisions with cylinders.
  
  // Compute the vector from the point mass to the closest point on the cylinder's central axis
  Vector3D direction = pm.position - origin;
  Vector3D closest_point_on_axis = origin + (dot(direction, axis) / axis.norm2()) * axis;
  
  // Compute the vector from the closest point on the cylinder's central axis to the point mass
  Vector3D to_closest_point = closest_point_on_axis - pm.position;
  
  if (to_closest_point.norm() < radius) {
    
    // Compute the correction vector needed to be applied to the point mass's last_position 
    // in order to reach the tangent point on the cylinder
    Vector3D tangent = direction - to_closest_point;
    Vector3D correction_vector = (tangent.unit() * radius) - to_closest_point;
    
    // Scale the correction vector by the friction coefficient and apply it to the point mass's position
    pm.position = pm.last_position + correction_vector * (1 - friction);
  }
}

void Cylinder::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the cylinder when rendered
  m_cylinder_mesh.draw_cylinder(shader, origin, radius * 0.92, height);
}
