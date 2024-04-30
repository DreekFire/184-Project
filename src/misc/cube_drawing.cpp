#include <cmath>
#include <nanogui/nanogui.h>

#include "cube_drawing.h"

#include "CGL/color.h"
#include "CGL/vector3D.h"

#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 3
#define VERTEX_OFFSET 0
#define TANGEN_OFFSET 8
#define VERTEX_SIZE 3

using namespace nanogui;


namespace CGL {
    namespace Misc {
        CubeMesh::CubeMesh(int size) {
            generate_cube();
            build_data();
        }


        void CubeMesh::generate_cube() {
            // Cube vertices
            Vertices = {
                // Front face
                -0.5, -0.5,  0.5, // Vertex 0
                 0.5, -0.5,  0.5, // Vertex 1
                 0.5,  0.5,  0.5, // Vertex 2
                -0.5,  0.5,  0.5, // Vertex 3
                // Back face
                -0.5, -0.5, -0.5, // Vertex 4
                 0.5, -0.5, -0.5, // Vertex 5
                 0.5,  0.5, -0.5, // Vertex 6
                -0.5,  0.5, -0.5  // Vertex 7
            };

            // Cube indices (triangles)
            Indices = {
                // Front face
                0, 1, 2, // Triangle 1
                2, 3, 0, // Triangle 2
                // Back face
                4, 5, 6, // Triangle 1
                6, 7, 4, // Triangle 2
                // Top face
                3, 2, 6, // Triangle 1
                6, 7, 3, // Triangle 2
                // Bottom face
                0, 1, 5, // Triangle 1
                5, 4, 0, // Triangle 2
                // Right face
                1, 5, 6, // Triangle 1
                6, 2, 1, // Triangle 2
                // Left face
                0, 4, 7, // Triangle 1
                7, 3, 0  // Triangle 2
            };
        }

        void CubeMesh::build_data() {

            positions = MatrixXf(4, Indices.size() * 3);
            normals = MatrixXf(4, Indices.size() * 3);

            for (size_t i = 0; i < Indices.size(); i += 3) {
                unsigned int i1 = Indices[i];
                unsigned int i2 = Indices[i + 1];
                unsigned int i3 = Indices[i + 2];

                Vector3D v1(Vertices[i1 * VERTEX_SIZE], Vertices[i1 * VERTEX_SIZE + 1], Vertices[i1 * VERTEX_SIZE + 2]);
                Vector3D v2(Vertices[i2 * VERTEX_SIZE], Vertices[i2 * VERTEX_SIZE + 1], Vertices[i2 * VERTEX_SIZE + 2]);
                Vector3D v3(Vertices[i3 * VERTEX_SIZE], Vertices[i3 * VERTEX_SIZE + 1], Vertices[i3 * VERTEX_SIZE + 2]);

                Vector3D normal = cross(v2 - v1, v3 - v1).unit();

                for (int j = 0; j < 3; j++) {
                    positions.col(i * 3 + j) << Vertices[Indices[i + j] * VERTEX_SIZE], Vertices[Indices[i + j] * VERTEX_SIZE + 1], Vertices[Indices[i + j] * VERTEX_SIZE + 2], 1.0;
                    normals.col(i * 3 + j) << normal.x, normal.y, normal.z, 0.0;
                }
            }

        }

        void CubeMesh::draw_cube(GLShader& shader, const Vector3D& p, double size) {
            Matrix4f model;
            model << size, 0, 0, p.x,
                0, size, 0, p.y,
                0, 0, size, p.z,
                0, 0, 0, 1;

            shader.setUniform("u_model", model);

            shader.uploadAttrib("in_position", positions);
            if (shader.attrib("in_normal", false) != -1) {
                shader.uploadAttrib("in_normal", normals);
            }

            shader.drawArray(GL_TRIANGLES, 0, Indices.size() * 3);
        }

    } // namespace Misc
} // namespace CGL
