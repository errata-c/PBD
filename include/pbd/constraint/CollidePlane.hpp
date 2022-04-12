#pragma once
#include <glm/vec3.hpp>

namespace pbd {
	class Engine3D;
	struct CollidePlane {
		glm::vec3 origin, normal;

		void eval(Engine3D& engine, int id) const;
	};
}