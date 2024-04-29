#include <cmath>
#include <nanogui/nanogui.h>

#include "./cylinder_drawing.h"


#include "CGL/color.h"
#include "CGL/vector3D.h"

#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5
#define TANGEN_OFFSET 8
#define VERTEX_SIZE 11


using namespace nanogui;

namespace CGL {
namespace Misc {

CylinderMesh::CylinderMesh(int num_slices, int num_stacks)
: cylinder_num_slices(num_slices)
, cylinder_num_stacks(num_stacks)
, cylinder_num_vertices((num_slices + 1) * (num_stacks + 1))
, cylinder_num_indices(6 * num_slices * num_stacks) {
  
  Indices.resize(cylinder_num_indices);
  Vertices.resize(VERTEX_SIZE * cylinder_num_vertices);
  
  double stack_height = 1.0 / num_stacks;
  double slice_angle = 2 * M_PI / num_slices;

  for (int i = 0; i <= num_stacks; i++) {
    for (int j = 0; j <= num_slices; j++) {
      double u = ((double)j) / num_slices;
      double v = ((double)i) / num_stacks;
      
      double angle = j * slice_angle;
      double x = cos(angle);
      double z = sin(angle);
      
      double y = v * 2 - 1;  // scaling to [-1, 1]

      double *vptr = &Vertices[VERTEX_SIZE * c_index(i, j)];

      vptr[TCOORD_OFFSET + 0] = u;
      vptr[TCOORD_OFFSET + 1] = v;
      
      vptr[VERTEX_OFFSET + 0] = x;
      vptr[VERTEX_OFFSET + 1] = y;
      vptr[VERTEX_OFFSET + 2] = z;

      vptr[NORMAL_OFFSET + 0] = x;
      vptr[NORMAL_OFFSET + 1] = 0;
      vptr[NORMAL_OFFSET + 2] = z;
      
      vptr[TANGEN_OFFSET + 0] = -z;
      vptr[TANGEN_OFFSET + 1] = 0;
      vptr[TANGEN_OFFSET + 2] = x;
    }
  }

  for (int i = 0; i < num_stacks; i++) {
    for (int j = 0; j < num_slices; j++) {
      unsigned int *iptr = &Indices[6 * (num_slices * i + j)];

      unsigned int i00 = c_index(i, j);
      unsigned int i10 = c_index(i + 1, j);
      unsigned int i11 = c_index(i + 1, j + 1);
      unsigned int i01 = c_index(i, j + 1);

      iptr[0] = i00;
      iptr[1] = i10;
      iptr[2] = i11;
      iptr[3] = i11;
      iptr[4] = i01;
      iptr[5] = i00;
    }
  }
  
  build_data();
}

int CylinderMesh::c_index(int stack, int slice) {
  return ((stack) * (cylinder_num_slices + 1) + (slice));
}

void CylinderMesh::build_data() {
  
  positions = MatrixXf(4, cylinder_num_indices * 3);
  normals = MatrixXf(4, cylinder_num_indices * 3);
  uvs = MatrixXf(2, cylinder_num_indices * 3);
  tangents = MatrixXf(4, cylinder_num_indices * 3);

  for (int i = 0; i < cylinder_num_indices; i += 3) {
    double *vPtr1 = &Vertices[VERTEX_SIZE * Indices[i]];
    double *vPtr2 = &Vertices[VERTEX_SIZE * Indices[i + 1]];
    double *vPtr3 = &Vertices[VERTEX_SIZE * Indices[i + 2]];

    Vector3D p1(vPtr1[VERTEX_OFFSET], vPtr1[VERTEX_OFFSET + 1],
                vPtr1[VERTEX_OFFSET + 2]);
    Vector3D p2(vPtr2[VERTEX_OFFSET], vPtr2[VERTEX_OFFSET + 1],
                vPtr2[VERTEX_OFFSET + 2]);
    Vector3D p3(vPtr3[VERTEX_OFFSET], vPtr3[VERTEX_OFFSET + 1],
                vPtr3[VERTEX_OFFSET + 2]);

    Vector3D n1(vPtr1[NORMAL_OFFSET], vPtr1[NORMAL_OFFSET + 1],
                vPtr1[NORMAL_OFFSET + 2]);
    Vector3D n2(vPtr2[NORMAL_OFFSET], vPtr2[NORMAL_OFFSET + 1],
                vPtr2[NORMAL_OFFSET + 2]);
    Vector3D n3(vPtr3[NORMAL_OFFSET], vPtr3[NORMAL_OFFSET + 1],
                vPtr3[NORMAL_OFFSET + 2]);
        
    Vector3D uv1(vPtr1[TCOORD_OFFSET], vPtr1[TCOORD_OFFSET + 1], 0);
    Vector3D uv2(vPtr2[TCOORD_OFFSET], vPtr2[TCOORD_OFFSET + 1], 0);
    Vector3D uv3(vPtr3[TCOORD_OFFSET], vPtr3[TCOORD_OFFSET + 1], 0);
    
    Vector3D t1(vPtr1[TANGEN_OFFSET], vPtr1[TANGEN_OFFSET + 1],
                vPtr1[TANGEN_OFFSET + 2]);
    Vector3D t2(vPtr2[TANGEN_OFFSET], vPtr2[TANGEN_OFFSET + 1],
                vPtr2[TANGEN_OFFSET + 2]);
    Vector3D t3(vPtr3[TANGEN_OFFSET], vPtr3[TANGEN_OFFSET + 1],
                vPtr3[TANGEN_OFFSET + 2]);

    positions.col(i    ) << p1.x, p1.y, p1.z, 1.0;
    positions.col(i + 1) << p2.x, p2.y, p2.z, 1.0;
    positions.col(i + 2) << p3.x, p3.y, p3.z, 1.0;

    normals.col(i    ) << n1.x, n1.y, n1.z, 0.0;
    normals.col(i + 1) << n2.x, n2.y, n2.z, 0.0;
    normals.col(i + 2) << n3.x, n3.y, n3.z, 0.0;
    
    uvs.col(i    ) << uv1.x, uv1.y;
    uvs.col(i + 1) << uv2.x, uv2.y;
    uvs.col(i + 2) << uv3.x, uv3.y;
    
    tangents.col(i    ) << t1.x, t1.y, t1.z, 0.0;
    tangents.col(i + 1) << t2.x, t2.y, t2.z, 0.0;
    tangents.col(i + 2) << t3.x, t3.y, t3.z, 0.0;
  }
}

void CylinderMesh::draw_cylinder(GLShader &shader, const Vector3D &axis, const Vector3D &p, double r, double h) {
    std::cout << "axis: " << axis << std::endl; 
    std::cout << "position: " << p << std::endl; 

  Vector3D z_axis = axis.unit();
  Vector3D x_axis = cross(Vector3D(0, 1, 0), z_axis).unit();
  Vector3D y_axis = cross(z_axis, x_axis).unit();

  /*std::cout << "x_axis: " << x_axis << std::endl;
  std::cout << "y_axis: " << y_axis << std::endl;
  std::cout << "z_axis: " << z_axis << std::endl; */

  Matrix4f model;
  model << r * x_axis.x, r * y_axis.x, r * z_axis.x, p.x,
           r * x_axis.y, r * y_axis.y, r * z_axis.y, p.y,
           r * x_axis.z, r * y_axis.z, r * z_axis.z, p.z,
           0,             0,             0,             1;

  std::cout << "Model Matrix:" << std::endl << model << std::endl;

  //scale the cylinder by the height 
  model(1, 1) *= h;

  shader.setUniform("u_model", model);

  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }
  if (shader.attrib("in_uv", false) != -1) {
    shader.uploadAttrib("in_uv", uvs);
  }
  if (shader.attrib("in_tangent", false) != -1) {
    shader.uploadAttrib("in_tangent", tangents, false);
  }

  shader.drawArray(GL_TRIANGLES, 0, cylinder_num_indices);
}


} // namespace Misc
} // namespace CGL
