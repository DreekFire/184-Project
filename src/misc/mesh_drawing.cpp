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

            // Calculate normals and tangents per vertex
            // Initialize accumulators for normals and tangents
            std::vector<Vector3D> vertexNormals(vertices.size(), Vector3D(0, 0, 0));
            std::vector<Vector3D> vertexTangents(vertices.size(), Vector3D(0, 0, 0));

            for (size_t i = 0; i < indices.size(); i += 3) {
                // Calculate face normal
                Vector3D p1(vertices[VERTEX_SIZE * indices[i] + VERTEX_OFFSET],
                    vertices[VERTEX_SIZE * indices[i] + VERTEX_OFFSET + 1],
                    vertices[VERTEX_SIZE * indices[i] + VERTEX_OFFSET + 2]);
                Vector3D p2(vertices[VERTEX_SIZE * indices[i + 1] + VERTEX_OFFSET],
                    vertices[VERTEX_SIZE * indices[i + 1] + VERTEX_OFFSET + 1],
                    vertices[VERTEX_SIZE * indices[i + 1] + VERTEX_OFFSET + 2]);
                Vector3D p3(vertices[VERTEX_SIZE * indices[i + 2] + VERTEX_OFFSET],
                    vertices[VERTEX_SIZE * indices[i + 2] + VERTEX_OFFSET + 1],
                    vertices[VERTEX_SIZE * indices[i + 2] + VERTEX_OFFSET + 2]);
                Vector3D faceNormal = cross(p2 - p1, p3 - p1).unit();

                // Accumulate normals and tangents for each vertex of the face
                for (int j = 0; j < 3; ++j) {
                    int vertexIndex = indices[i + j];
                    vertexNormals[vertexIndex] += faceNormal;
                    // Compute tangent
                    Vector3D edge1 = p2 - p1;
                    Vector3D edge2 = p3 - p1;
                    Vector3D deltaUV1(vertices[VERTEX_SIZE * indices[i + 1] + TCOORD_OFFSET] - vertices[VERTEX_SIZE * indices[i] + TCOORD_OFFSET],
                        						vertices[VERTEX_SIZE * indices[i + 1] + TCOORD_OFFSET + 1] - vertices[VERTEX_SIZE * indices[i] + TCOORD_OFFSET + 1], 0);
                    Vector3D deltaUV2(vertices[VERTEX_SIZE * indices[i + 2] + TCOORD_OFFSET] - vertices[VERTEX_SIZE * indices[i] + TCOORD_OFFSET],
                        						vertices[VERTEX_SIZE * indices[i + 2] + TCOORD_OFFSET + 1] - vertices[VERTEX_SIZE * indices[i] + TCOORD_OFFSET + 1], 0);
                    float f = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
                    Vector3D tangent(f * (deltaUV2.y * edge1.x - deltaUV1.y * edge2.x),
                        												f * (deltaUV2.y * edge1.y - deltaUV1.y * edge2.y),
                        												f * (deltaUV2.y * edge1.z - deltaUV1.y * edge2.z));
                    vertexTangents[vertexIndex] += tangent;
                }
            }

            // Normalize normals and tangents
            for (size_t i = 0; i < vertexNormals.size(); ++i)
                vertexNormals[i].normalize();
            // Similarly, normalize tangents
            for (size_t i = 0; i < vertexTangents.size(); ++i)
				vertexTangents[i].normalize();

            // Populate matrices
            for (size_t i = 0; i < indices.size(); i += 3) {
                for (int j = 0; j < 3; ++j) {
                    int vertexIndex = indices[i + j];

                    // Copy positions
                    positions.col(i + j) << vertices[VERTEX_SIZE * vertexIndex + VERTEX_OFFSET],
                        vertices[VERTEX_SIZE * vertexIndex + VERTEX_OFFSET + 1],
                        vertices[VERTEX_SIZE * vertexIndex + VERTEX_OFFSET + 2],
                        1.0;

                    // Copy normals
                    normals.col(i + j) << vertexNormals[vertexIndex].x,
                        vertexNormals[vertexIndex].y,
                        vertexNormals[vertexIndex].z,
                        0.0;

                    // Copy UVs
                    uvs.col(i + j) << vertices[VERTEX_SIZE * vertexIndex + TCOORD_OFFSET],
                        vertices[VERTEX_SIZE * vertexIndex + TCOORD_OFFSET + 1];

                    // Copy tangents if needed
                     tangents.col(i + j) << vertexTangents[vertexIndex].x,
                                            vertexTangents[vertexIndex].y,
                                            vertexTangents[vertexIndex].z,
                                            0.0;
                }
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
            // find the relative location of the point mass in the mesh in terms of side length
            double x = pm.last_position.x + width / 2.0;
            double y = pm.last_position.z + height / 2.0;
            // find the triangle that the point mass is in
            int x1 = std::floor(x);
            int y1 = std::floor(y);
            // find the relative position of the point mass in the triangle
            double x2 = x - x1;
            double y2 = y - y1;
            // find the triangle that the point mass is in
            double* vPtr1 = &vertices[VERTEX_SIZE * (y1 * width + x1)];
            double* vPtr2 = &vertices[VERTEX_SIZE * ((y1 + 1) * width + x1)];
            double* vPtr3 = &vertices[VERTEX_SIZE * (y1 * width + x1 + 1)];

            Vector3D p1(vPtr1[VERTEX_OFFSET], vPtr1[VERTEX_OFFSET + 1], vPtr1[VERTEX_OFFSET + 2]);
            Vector3D p2(vPtr2[VERTEX_OFFSET], vPtr2[VERTEX_OFFSET + 1], vPtr2[VERTEX_OFFSET + 2]);
            Vector3D p3(vPtr3[VERTEX_OFFSET], vPtr3[VERTEX_OFFSET + 1], vPtr3[VERTEX_OFFSET + 2]);

            // todo: use normal vector for collision
            Vector3D normal = cross(p2 - p1, p3 - p1);
            if (normal.y < 0) {
                std::cout << "flip the normal Derek" << std::endl;
            }

            // find the closest point on the triangle to the point mass
            if (x2 + y2 < 1) {
				closest_point = p1 + x2 * (p3 - p1) + y2 * (p2 - p1);
			}
            else {
				closest_point = p3 + (1 - x2) * (p2 - p3) + (1 - y2) * (p1 - p3);
			}
            // if the point mass is below the mesh, move it to the closest point on the mesh
            if (pm.position.y <= closest_point.y + 0.001) {
                // pm.last_position = pm.position;
                pm.position = closest_point;
                // add adjustment factor to keep the point mass on the surface
                pm.position.y += 0.001;
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
