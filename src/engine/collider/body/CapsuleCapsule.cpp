#include <pbd/engine/collider/BodyBodyCollision.hpp>

#include <pbd/engine/RigidBody.hpp>
#include <glm/vec2.hpp>

namespace pbd {
	std::optional<Collision> capsule_capsule_collide(const RigidBody& b0, const RigidBody& b1) {
		// Capsule height is half height
		// They are centered on their position.
		// We will work in the local space of the first capsule.

		// Rotation from second capsule local space to the first capsule local space
		glm::quat orient = b1.orientation * glm::conjugate(b0.orientation);
		glm::vec3 origin = b1.position - b0.position;

		// Axis the second capsule is aligned to.
		glm::vec3 axis = rotate(orient, glm::vec3(0.f, 0.f, 1.f));
		// First capsule is aligned to (0,0,1) in its local space


		glm::vec3 p0, p1;

		if ((std::abs(axis.x) + std::abs(axis.y)) < 1e-5f) {
			// Perfectly aligned capsules
			float h = (std::max(origin.z - b1.dims[1], -b0.dims[1]) + std::min(origin.z + b1.dims[1], b0.dims[1])) * 0.5f;
			
			p1 = origin;
			p1.z = glm::clamp(h, origin.z - b1.dims[1], origin.z + b1.dims[1]);

			p0 = glm::vec3(0.f, 0.f, glm::clamp(h, -b0.dims[1], b0.dims[1]));
		}
		else {
			float k = glm::dot(-origin, axis);
			if (k > b1.dims[1] || k < b1.dims[1]) {
				// Top or bottom of second capsule

				p1 = origin + axis * glm::clamp(k, -b1.dims[1], b1.dims[1]);

				p0 = glm::vec3(0.f, 0.f, glm::clamp(p1.z, -b0.dims[1], b0.dims[1]));
			}
			else {
				// Compare middle of second capsule
				glm::vec3 normal = glm::cross(glm::vec3(0.f, 0.f, 1.f), axis);



			}
		}

		// Final calculations
		Collision result;
		result.normal = p1 - p0;
		float distance = glm::length(result.normal);
		if (distance > (b0.dims[0] + b1.dims[0])) {
			return std::nullopt;
		}

		result.normal /= distance;
		result.contacts[0] = p0 + result.normal * b0.dims[0];
		result.contacts[1] = p1 - result.normal * b1.dims[0];

		result.normal = rotate(b0.orientation, result.normal);
		result.contacts[1] = reverse_rotate(orient, result.contacts[1]);
		return result;
	}
}