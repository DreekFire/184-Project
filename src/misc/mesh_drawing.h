#ifndef CGL_UTIL_MESHDRAWING_H
#define CGL_UTIL_MESHDRAWING_H

#include <vector>
#include <nanogui/nanogui.h>
#include "CGL/CGL.h"
#include "../clothMesh.h"

using namespace nanogui;

namespace CGL {
	namespace Misc {

        class MeshDrawing {
        public:
            MeshDrawing(const std::string& imagePath, const std::string& project_root);

            void loadImage();
            void generateIndices();
            void drawMesh(GLShader& shader, const Vector3D& position, float scale);
            void buildData();
            void collide(PointMass& pm);

            std::vector<unsigned int> indices;
            std::vector<double> vertices;

            MatrixXf positions;
            MatrixXf normals;
            MatrixXf uvs;
            MatrixXf tangents;
            int width, height;

        private:
            std::string imagePath;
            std::string projectRoot;
        };


	} // namespace Misc
} // namespace CGL

#endif // CGL_UTIL_MESHDRAWING_H
