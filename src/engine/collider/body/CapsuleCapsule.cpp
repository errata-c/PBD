#include <pbd/engine/collider/BodyBodyCollision.hpp>

#include <pbd/engine/RigidBody.hpp>
#include <pbd/common/Utils.hpp>
#include <glm/vec2.hpp>

namespace pbd {
	std::optional<Collision> capsule_capsule_collide(const RigidBody& b0, const RigidBody& b1) {
		// Capsule height is half height
		// They are centered on their position.
		// We will work in the local space of the first capsule.

		// Rotation from second capsule local space to the first capsule local space
		quat_t orient = b1.orientation * glm::conjugate(b0.orientation);
		vec3_t origin = rotate(glm::conjugate(b0.orientation), b1.position - b0.position);

		// Axis the second capsule is aligned to.
		// As long as the orientations of the bodies are normalized, this axis is as well.
		vec3_t axis = rotate(orient, vec3_t(0.f, 0.f, 1.f));
		// First capsule is aligned to (0,0,1) in its local space

		real_t rr = b0.dims.x + b1.dims.x;

		vec3_t p0, p1;

		if ((std::abs(axis.x) + std::abs(axis.y)) < 1e-5f) {
			// Perfectly aligned capsules

			// Early exit, cylinders cannot possibly collide
			if (glm::length2(vec2_t(origin.x, origin.y)) > rr * rr) {
				return std::nullopt;
			}

			real_t h = (std::max(origin.z - b1.dims[1], -b0.dims[1]) + std::min(origin.z + b1.dims[1], b0.dims[1])) * 0.5f;
			
			p1 = origin;
			p1.z = glm::clamp(h, origin.z - b1.dims[1], origin.z + b1.dims[1]);

			p0 = vec3_t(0.f, 0.f, glm::clamp(h, -b0.dims[1], b0.dims[1]));
		}
		else {
			// Misaligned.

			vec3_t v1(axis.x * axis.z, axis.z * axis.y, -axis.y * axis.y - axis.x * axis.x);

			real_t numer = glm::dot(-origin, v1);
			real_t denom = -v1.z;

			// Because they are misaligned, denom is always non-zero
			real_t t = numer / denom;
			// We must clamp 't' BEFORE we calculate 'u'
			t = glm::clamp(t, -b0.dims.y, b0.dims.y);

			// We use 't' to calculate 'u'
			real_t u = glm::dot(axis, vec3_t(-origin.x, -origin.y, t-origin.z));
			u = glm::clamp(u, -b1.dims.y, b1.dims.y);

			// The final point locations.
			p1 = origin + axis * u;
			p0 = vec3_t(0.f, 0.f, t);
		}

		// Final calculations
		Collision result;

		// It is technically possible, but unlikely, that p0 and p1 are equal.
		result.normal = p1 - p0;
		real_t distance = glm::length2(result.normal);
		if (distance > rr * rr) {
			return std::nullopt;
		}
		distance = std::sqrt(distance);

		if (distance < 1e-5f) {
			// This is a weird edge case, that I'm not entirely sure is worth handling.
			return std::nullopt;
		}
		else {
			result.normal /= distance;
		}

		result.contacts[0] = p0 + result.normal * b0.dims[0];
		result.contacts[1] = p1 - result.normal * b1.dims[0];

		result.normal = pbd::rotate(b0.orientation, result.normal);
		result.contacts[1] = pbd::reverse_rotate(orient, result.contacts[1]);
		return result;
	}
}