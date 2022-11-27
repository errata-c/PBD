#pragma once
#include <array>
#include <optional>
#include <glm/vec3.hpp>

namespace pbd {
	struct Collision {
		std::array<glm::vec3, 2> contacts;
		glm::vec3 normal;
		float depth;
	};
}