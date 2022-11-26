#include <pbd/common/Utils.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	glm::vec3 rotate(const glm::quat& orientation, const glm::vec3& v) {
		glm::quat tmp(0.f, v.x, v.y, v.z);
		tmp = orientation * tmp * glm::conjugate(orientation);
		return glm::vec3{ tmp.x, tmp.y, tmp.z };
	}
	glm::vec3 reverse_rotate(const glm::quat& orientation, const glm::vec3& v) {
		glm::quat tmp(0.f, v.x, v.y, v.z);
		tmp = glm::conjugate(orientation) * tmp * orientation;
		return glm::vec3{ tmp.x, tmp.y, tmp.z };
	}

	glm::vec3 perpendicular(const glm::vec3& x, const glm::vec3& n) {
		return x - n * glm::dot(x, n);
	}

	glm::vec3 friction_delta(const glm::vec3& perp, float overlap, float sfriction, float kfriction) {
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

	float tetrahedron_volume(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) {
		glm::mat3 form{
			p1 - p0,
			p2 - p0,
			p3 - p0
		};

		return std::abs(glm::determinant(form)) / 6.f;
	}
}