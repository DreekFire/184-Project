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
                    double lon = ((double)x) / side_length;
                    double lat = ((double)y) / side_length;

                    double* vptr = &vertices[VERTEX_SIZE * (y * side_length + x)];

                    // Texture coordinates
                    vptr[TCOORD_OFFSET + 0] = lon;
                    vptr[TCOORD_OFFSET + 1] = lat;

                    // Position
                    vptr[VERTEX_OFFSET + 0] = x - side_length / 2.0;
                    vptr[VERTEX_OFFSET + 1] = grayscale_value * 5.0 / 255.0;
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

            for (int i = 0; i < indices.size(); i += 3) {
                double* vPtr1 = &vertices[VERTEX_SIZE * indices[i]];
                double* vPtr2 = &vertices[VERTEX_SIZE * indices[i + 1]];
                double* vPtr3 = &vertices[VERTEX_SIZE * indices[i + 2]];

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

                positions.col(i) << p1.x, p1.y, p1.z, 1.0;
                positions.col(i + 1) << p2.x, p2.y, p2.z, 1.0;
                positions.col(i + 2) << p3.x, p3.y, p3.z, 1.0;

                normals.col(i) << n1.x, n1.y, n1.z, 0.0;
                normals.col(i + 1) << n2.x, n2.y, n2.z, 0.0;
                normals.col(i + 2) << n3.x, n3.y, n3.z, 0.0;

                uvs.col(i) << uv1.x, uv1.y;
                uvs.col(i + 1) << uv2.x, uv2.y;
                uvs.col(i + 2) << uv3.x, uv3.y;

                tangents.col(i) << t1.x, t1.y, t1.z, 0.0;
                tangents.col(i + 1) << t2.x, t2.y, t2.z, 0.0;
                tangents.col(i + 2) << t3.x, t3.y, t3.z, 0.0;
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
