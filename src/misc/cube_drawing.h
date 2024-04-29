#ifndef CGL_UTIL_CUBEDRAWING_H
#define CGL_UTIL_CUBEDRAWING_H

#include <vector>
#include <nanogui/nanogui.h>
#include "CGL/CGL.h"
#include "CGL/Vector3D.h"

using namespace nanogui;

namespace CGL {
	namespace Misc {

		class CubeMesh {
		public:
			CubeMesh(int size);
			void draw_cube(GLShader& shader, const Vector3D& p, double size);

		private:
			std::vector<unsigned int> Indices;
			std::vector<double> Vertices;
			std::vector<double> TexCoords;
			MatrixXf positions;
			MatrixXf normals;
			MatrixXf texcoords;

			void build_data();
			void generate_cube();
		};

	} // namespace Misc
} // namespace CGL

#endif // CGL_UTIL_CUBEDRAWING_H
