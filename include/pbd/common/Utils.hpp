#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>

namespace pbd {
	class Engine;

	glm::vec3 perpendicular(const glm::vec3 & x, const glm::vec3 & n);

	glm::vec3 frictionDelta(const glm::vec3 & perp, float overlap, float sfriction, float kfriction);
}