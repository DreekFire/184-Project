#ifndef CGL_UTIL_MESHDRAWING_H
#define CGL_UTIL_MESHDRAWING_H

#include <vector>
#include <nanogui/nanogui.h>
#include "CGL/CGL.h"

using namespace nanogui;

namespace CGL {
	namespace Misc {

        class MeshDrawing {
        public:
            MeshDrawing(const std::string& imagePath);

            void loadImage();
            void generateIndices();
            void drawMesh(GLShader& shader, const Vector3D& position, float scale);
            void buildData();

            std::vector<unsigned int> indices;
            std::vector<double> vertices;

            MatrixXf positions;
            MatrixXf normals;
            MatrixXf uvs;
            MatrixXf tangents;
            int width, height;

        private:
            std::string imagePath;
        };


	} // namespace Misc
} // namespace CGL

#endif // CGL_UTIL_MESHDRAWING_H
