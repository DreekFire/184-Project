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
			// Supply the desired texture and dimensions
			MeshDrawing(const char* Path, int width, int height);

			/**
			 * Draws a mesh with the given position and scale in OpenGL, using the
			 * current modelview/projection matrices and color/material settings.
			 */
			void draw_mesh(GLShader& shader, const Vector3D& position, double scale);

		private:
			std::vector<unsigned int> indices;
			std::vector<Vector3D> vertices;
			std::vector<Vector3D> normals;

			int textureWidth;
			int textureHeight;

			GLuint textureID; // Texture ID

			void loadTexture(const char* texturePath);
			void generateMeshFromTexture(const char* texturePath);
		};

	} // namespace Misc
} // namespace CGL

#endif // CGL_UTIL_MESHDRAWING_H
