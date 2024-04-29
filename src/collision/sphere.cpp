#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.

  // if ((pm.position - origin).norm() < radius) {

  //   //Compute the correction vector needed to be applied to the point mass's last_position in order to reach the tangent point.
  //   Vector3D distance_to_origin = pm.last_position - origin;
  //   Vector3D correction_vector = (distance_to_origin.unit() * radius + origin) - pm.last_position;

  //   //Scale the correction vector by the friction coefficient and apply it to the point mass's position. 
  //   pm.position = pm.last_position + correction_vector * (1 - friction);
  // }
  // return;

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
