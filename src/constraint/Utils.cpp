#include <pbd/common/Utils.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	glm::vec3 perpendicular(const glm::vec3& x, const glm::vec3& n) {
		return x - n * glm::dot(x, n);
	}

	glm::vec3 frictionDelta(const glm::vec3& perp, float overlap, float sfriction, float kfriction) {
		float plen = glm::length(perp);
		if (plen < (sfriction * overlap)) {
			// Completely eliminate tangential motion
			return -perp;
		}
		else {
			// Damp tangential motion
			return -perp * std::min((kfriction * overlap) / plen, 1.f);
		}
	}
}