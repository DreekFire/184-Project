#include <cmath>
#include <nanogui/nanogui.h>

#include "mesh_drawing.h"

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

        MeshDrawing::MeshDrawing(const char* texturePath, int width, int height)
            : textureWidth(width)
            , textureHeight(height) {
            loadTexture(texturePath);
            generateMeshFromTexture(texturePath);
        }

        void MeshDrawing::loadTexture(const char* texturePath) {
            // Load texture using Nanogui
            int channels;
            unsigned char* data = stbi_load(texturePath, &textureWidth, &textureHeight, &channels, STBI_rgb_alpha);
            if (!data) {
                std::cerr << "Failed to load texture: " << texturePath << std::endl;
                return;
            }

            // Generate texture ID
            glGenTextures(1, &textureID);
            glBindTexture(GL_TEXTURE_2D, textureID);

            // Set texture parameters
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // Upload texture data
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureWidth, textureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

            // Free texture data
            stbi_image_free(data);
        }

        void MeshDrawing::generateMeshFromTexture(const char* texturePath) {
            // Load texture using Nanogui
            int channels;
            unsigned char* data = stbi_load(texturePath, &textureWidth, &textureHeight, &channels, STBI_rgb_alpha);
            if (!data) {
                std::cerr << "Failed to load texture: " << texturePath << std::endl;
                return;
            }

            // Clear previous data
            vertices.clear();
            normals.clear();
            texCoords.clear();

            // Generate mesh from texture data
            for (int y = 0; y < textureHeight; ++y) {
                for (int x = 0; x < textureWidth; ++x) {
                    // Assuming texture is grayscale, use the red channel value as height
                    float heightValue = data[(y * textureWidth + x) * 4] / 255.0f; // Normalize to [0, 1]

                    // Generate vertex
                    Vector3D vertex(x, heightValue, y); // Adjust heightValue to your scale
                    vertices.push_back(vertex);

                    // Generate texture coordinate
                    Vector2D texCoord((float)x / (float)(textureWidth - 1), (float)y / (float)(textureHeight - 1)); // Texture coordinate
                    texCoords.push_back(texCoord);
                }
            }

            // Generate normals
            for (int y = 0; y < textureHeight; ++y) {
                for (int x = 0; x < textureWidth; ++x) {
                    // Calculate normals using neighboring vertices
                    Vector3D center = vertices[y * textureWidth + x];
                    Vector3D left = (x > 0) ? vertices[y * textureWidth + (x - 1)] : center;
                    Vector3D right = (x < textureWidth - 1) ? vertices[y * textureWidth + (x + 1)] : center;
                    Vector3D up = (y > 0) ? vertices[(y - 1) * textureWidth + x] : center;
                    Vector3D down = (y < textureHeight - 1) ? vertices[(y + 1) * textureWidth + x] : center;

                    Vector3D normal = cross(right - left, up - down).unit();
                    normals.push_back(normal);
                }
            }

            // Free texture data
            stbi_image_free(data);
        }

        void MeshDrawing::draw_mesh(GLShader& shader, const Vector3D& position, double scale) {
            Matrix4f model;
            model << scale, 0, 0, position.x, 0, scale, 0, position.y, 0, 0, scale, position.z, 0, 0, 0, 1;

            shader.setUniform("u_model", model);

            glBindTexture(GL_TEXTURE_2D, textureID);

            if (!vertices.empty() && !normals.empty() && !texCoords.empty()) {
                if (shader.attrib("in_position", false) != -1) {
                    shader.uploadAttrib("in_position", vertices);
                }
                if (shader.attrib("in_normal", false) != -1) {
                    shader.uploadAttrib("in_normal", normals);
                }
                if (shader.attrib("in_texcoord", false) != -1) {
                    shader.uploadAttrib("in_texcoord", texCoords);
                }

                shader.drawArray(GL_TRIANGLES, 0, vertices.size());
            }
        }

    } // namespace Misc
} // namespace CGL