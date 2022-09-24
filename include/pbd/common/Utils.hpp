#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>

namespace pbd {
	class Engine;

	glm::vec3 perpendicular(const glm::vec3 & x, const glm::vec3 & n);

	glm::vec3 friction_delta(const glm::vec3 & perp, float overlap, float sfriction, float kfriction);

	float tetrahedron_volume(const glm::vec3 & p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3);
}