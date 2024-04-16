#ifndef COLLISIONOBJECT_CYLINDER_H
#define COLLISIONOBJECT_CYLINDER_H

#include "../clothMesh.h"
#include "../misc/cylinder_drawing.h"
#include "collisionObject.h"

using namespace CGL;
using namespace std;

struct Cylinder : public CollisionObject {
public:
  Cylinder(const Vector3D &origin, const Vector3D &axis, double radius, double height, double friction, int num_slices = 40, int num_stacks = 40)
      : origin(origin), axis(axis.unit()), radius(radius), height(height), radius2(radius * radius),
        friction(friction), m_cylinder_mesh(Misc::CylinderMesh(num_slices, num_stacks)) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);

private:
  Vector3D origin;
  Vector3D axis;  // Central axis of the cylinder
  double radius;
  double height;
  double radius2;

  double friction;
  
  Misc::CylinderMesh m_cylinder_mesh;
};

#endif /* COLLISIONOBJECT_CYLINDER_H */
