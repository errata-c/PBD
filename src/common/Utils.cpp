#include <pbd/common/Utils.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	vec3_t rotate(const quat_t& orientation, const vec3_t& v) {
		quat_t tmp(0.f, v.x, v.y, v.z);
		tmp = orientation * tmp * glm::conjugate(orientation);
		return vec3_t{ tmp.x, tmp.y, tmp.z };
	}
	vec3_t reverse_rotate(const quat_t& orientation, const vec3_t& v) {
		quat_t tmp(0.f, v.x, v.y, v.z);
		tmp = glm::conjugate(orientation) * tmp * orientation;
		return vec3_t{ tmp.x, tmp.y, tmp.z };
	}

	vec3_t perpendicular(const vec3_t& x, const vec3_t& n) {
		return x - n * glm::dot(x, n);
	}

	vec3_t friction_delta(const vec3_t& perp, real_t overlap, real_t sfriction, real_t kfriction) {
		real_t plen = glm::length(perp);
		if (plen < (sfriction * overlap)) {
			// Completely eliminate tangential motion
			return -perp;
		}
		else {
			// Damp tangential motion
			return -perp * std::min((kfriction * overlap) / plen, 1.f);
		}
	}

	real_t tetrahedron_volume(const vec3_t& p0, const vec3_t& p1, const vec3_t& p2, const vec3_t& p3) {
		glm::mat3 form{
			p1 - p0,
			p2 - p0,
			p3 - p0
		};

		return std::abs(glm::determinant(form)) / 6.f;
	}
}