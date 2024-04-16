#include <cmath>
#include <nanogui/nanogui.h>

#include "file_utils.h"
#include "stb_image.h"

#include "mesh_drawing.h"

#include "CGL/color.h"
#include "CGL/vector3D.h"

#include <iostream>
#include <filesystem>

#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5
#define TANGEN_OFFSET 8
#define VERTEX_SIZE 11

using namespace nanogui;
using namespace std;

namespace CGL {
    namespace Misc {

        MeshDrawing::MeshDrawing(const std::string& imagePath, const std::string& project_root)
            : imagePath(imagePath), projectRoot(project_root){
            loadImage();
        }

        void MeshDrawing::loadImage() {
            int channels;
            //. print the current path
            std::string exePath = projectRoot;
            std::string fullPath = exePath + imagePath;
            // print the path
            std::cout << "Path is: " << fullPath << std::endl;
            unsigned char* image_data = stbi_load(fullPath.c_str(), &width, &height, &channels, 1);
            if (!image_data) {
                std::cerr << "Error: Failed to load image." << std::endl;
                // just use a flat black texture as a fallback
                width = height = 512;
                image_data = new unsigned char[width * height];
                for (int i = 0; i < width * height; ++i) {
					image_data[i] = 0;
				}
            }


            // Assuming square image
            int side_length = std::min(width, height);

            vertices.resize(VERTEX_SIZE * side_length * side_length);

            for (int y = 0; y < side_length; ++y) {
                for (int x = 0; x < side_length; ++x) {
                    // Assuming image is grayscale
                    double grayscale_value = image_data[y * width + x] / 255.0;
                    // invert such that the dark areas are subtracted from the height
                    grayscale_value = 1.0 - grayscale_value;
                    double lon = ((double)x) / side_length;
                    double lat = ((double)y) / side_length;

                    double* vptr = &vertices[VERTEX_SIZE * (y * side_length + x)];

                    // Texture coordinates
                    vptr[TCOORD_OFFSET + 0] = lon;
                    vptr[TCOORD_OFFSET + 1] = lat;

                    // Position
                    vptr[VERTEX_OFFSET + 0] = x - side_length / 2.0;
                    vptr[VERTEX_OFFSET + 1] = (grayscale_value * 15.0);
                    vptr[VERTEX_OFFSET + 2] = y - side_length / 2.0;

                    // Normal (same as position in this case)
                    vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0];
                    vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1];
                    vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2];

                    // Tangent
                    vptr[TANGEN_OFFSET + 0] = 1.0;
                    vptr[TANGEN_OFFSET + 1] = 0.0;
                    vptr[TANGEN_OFFSET + 2] = 0.0;
                }
            }

            // offset the z height of all vertices such that the center of the mesh is at the origin
            // first find the height of the center of the mesh
            double* center_vertex = &vertices[VERTEX_SIZE * (side_length / 2 * side_length + side_length / 2)];
            double center_height = center_vertex[VERTEX_OFFSET + 1];
            for (int y = 0; y < side_length; ++y) {
                for (int x = 0; x < side_length; ++x) {
					double* vptr = &vertices[VERTEX_SIZE * (y * side_length + x)];
					vptr[VERTEX_OFFSET + 1] -= center_height;
				}
			}
            

            width = height = side_length;

            generateIndices();
            buildData();
        }

        void MeshDrawing::generateIndices() {
            indices.clear();
            for (int y = 0; y < height - 1; ++y) {
                for (int x = 0; x < width - 1; ++x) {
                    indices.push_back(y * width + x);
                    indices.push_back((y + 1) * width + x);
                    indices.push_back(y * width + x + 1);
                    indices.push_back((y + 1) * width + x);
                    indices.push_back((y + 1) * width + x + 1);
                    indices.push_back(y * width + x + 1);
                }
            }
        }

        void MeshDrawing::buildData() {
            positions = MatrixXf(4, indices.size() * 3);
            normals = MatrixXf(4, indices.size() * 3);
            uvs = MatrixXf(2, indices.size() * 3);
            tangents = MatrixXf(4, indices.size() * 3);

            for (size_t i = 0; i < indices.size(); i += 3) {
                double* vPtr1 = &vertices[VERTEX_SIZE * indices[i]];
                double* vPtr2 = &vertices[VERTEX_SIZE * indices[i + 1]];
                double* vPtr3 = &vertices[VERTEX_SIZE * indices[i + 2]];

                Vector3D p1(vPtr1[VERTEX_OFFSET], vPtr1[VERTEX_OFFSET + 1], vPtr1[VERTEX_OFFSET + 2]);
                Vector3D p2(vPtr2[VERTEX_OFFSET], vPtr2[VERTEX_OFFSET + 1], vPtr2[VERTEX_OFFSET + 2]);
                Vector3D p3(vPtr3[VERTEX_OFFSET], vPtr3[VERTEX_OFFSET + 1], vPtr3[VERTEX_OFFSET + 2]);

                // normals should be the normmal per vertex and not purely the position in space, calculate the normal of the triangle
                // normal at the vertex is the average of the normals of the triangles that share the vertex
                Vector3D n1 = cross(p2 - p1, p3 - p1).unit();
                Vector3D n2 = n1;
                Vector3D n3 = n1;

                Vector3D uv1(vPtr1[TCOORD_OFFSET], vPtr1[TCOORD_OFFSET + 1], 0);
                Vector3D uv2(vPtr2[TCOORD_OFFSET], vPtr2[TCOORD_OFFSET + 1], 0);
                Vector3D uv3(vPtr3[TCOORD_OFFSET], vPtr3[TCOORD_OFFSET + 1], 0);

                Vector3D t1(vPtr1[TANGEN_OFFSET], vPtr1[TANGEN_OFFSET + 1], vPtr1[TANGEN_OFFSET + 2]);
                Vector3D t2(vPtr2[TANGEN_OFFSET], vPtr2[TANGEN_OFFSET + 1], vPtr2[TANGEN_OFFSET + 2]);
                Vector3D t3(vPtr3[TANGEN_OFFSET], vPtr3[TANGEN_OFFSET + 1], vPtr3[TANGEN_OFFSET + 2]);

                // Copy positions
                positions.col(i) << p1.x, p1.y, p1.z, 1.0;
                positions.col(i + 1) << p2.x, p2.y, p2.z, 1.0;
                positions.col(i + 2) << p3.x, p3.y, p3.z, 1.0;

                // Copy normals
                normals.col(i) << n1.x, n1.y, n1.z, 0.0;
                normals.col(i + 1) << n2.x, n2.y, n2.z, 0.0;
                normals.col(i + 2) << n3.x, n3.y, n3.z, 0.0;

                // Copy UVs
                uvs.col(i) << uv1.x, uv1.y;
                uvs.col(i + 1) << uv2.x, uv2.y;
                uvs.col(i + 2) << uv3.x, uv3.y;

                // Copy tangents
                tangents.col(i) << t1.x, t1.y, t1.z, 0.0;
                tangents.col(i + 1) << t2.x, t2.y, t2.z, 0.0;
                tangents.col(i + 2) << t3.x, t3.y, t3.z, 0.0;
            }
        }

        // function to detect collisions and ensure that object remains on the surface
        void MeshDrawing::collide(PointMass& pm) {
            // pm has a position and a last_position
            // use the last position to detect collisions
			// find the closest point on the mesh to the point mass
            // it should be the triangle underneath the point mass
            // no looping since that would be too slow
            Vector3D closest_point;
            double closest_distance = std::numeric_limits<double>::infinity();
            // loop through each vertex of the mesh and find the closest one
            for (size_t i = 0; i < indices.size(); i += 3) {
				double* vPtr1 = &vertices[VERTEX_SIZE * indices[i]];
				double* vPtr2 = &vertices[VERTEX_SIZE * indices[i + 1]];
				double* vPtr3 = &vertices[VERTEX_SIZE * indices[i + 2]];

				Vector3D p1(vPtr1[VERTEX_OFFSET], vPtr1[VERTEX_OFFSET + 1], vPtr1[VERTEX_OFFSET + 2]);
				Vector3D p2(vPtr2[VERTEX_OFFSET], vPtr2[VERTEX_OFFSET + 1], vPtr2[VERTEX_OFFSET + 2]);
				Vector3D p3(vPtr3[VERTEX_OFFSET], vPtr3[VERTEX_OFFSET + 1], vPtr3[VERTEX_OFFSET + 2]);

				// find the closest point on the triangle to the point mass
				Vector3D closest = p1;
				Vector3D edge1 = p2 - p1;
				Vector3D edge2 = p3 - p1;
				Vector3D edge3 = p3 - p2;
				Vector3D normal = cross(edge1, edge2).unit();
				Vector3D to_pm = pm.last_position - p1;
				double d = dot(to_pm, normal);
				Vector3D projected = pm.last_position - d * normal;
				// check if the projected point is inside the triangle
				Vector3D c0 = cross(edge1, projected - p1);
				Vector3D c1 = cross(edge2, projected - p2);
				Vector3D c2 = cross(edge3, projected - p3);
                if (dot(c0, normal) >= 0 && dot(c1, normal) >= 0 && dot(c2, normal) >= 0) {
					// the projected point is inside the triangle
					double distance = (pm.last_position - projected).norm();
                    if (distance < closest_distance) {
						closest_distance = distance;
						closest_point = projected;
					}
				}
                else {
					// the projected point is outside the triangle
					// find the closest point on the edges of the triangle
					Vector3D closest_edge_point;
					double closest_edge_distance = std::numeric_limits<double>::infinity();
					Vector3D edge_points[3] = { p1, p2, p3 };
                }
            }
            
            // if the point mass is below the mesh, move it to the closest point on the mesh
            if (pm.last_position.y < closest_point.y) {
                pm.position = pm.last_position;
                // add adjustment factor to keep the point mass on the surface
                pm.position.y = closest_point.y + 0.0001;
            }
        }

        void MeshDrawing::drawMesh(GLShader& shader, const Vector3D& position, float scale) {
            Matrix4f model;
            model << scale, 0, 0, position.x,
				0, scale, 0, position.y,
				0, 0, scale, position.z,
				0, 0, 0, 1;

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

            shader.drawArray(GL_TRIANGLES, 0, indices.size());
        }


    } // namespace Misc
} // namespace CGL
