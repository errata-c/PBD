#pragma once
#include <glm/vec3.hpp>

namespace pbd {
	class Engine;
	struct CollidePlane {
		glm::vec3 origin, normal;

		void eval(Engine& engine, int id) const;
	};
}