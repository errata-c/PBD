#include <pbd/engine/collider/BodyBodyCollision.hpp>

#include <pbd/engine/RigidBody.hpp>

namespace pbd {
	std::optional<Collision> sphere_sphere_collide(const RigidBody& p0, const RigidBody& p1) {
		glm::vec3 normal = p1.position - p0.position;
		float d = glm::dot(normal, normal);
		float r2 = p0.dims[0] + p1.dims[0];
		if (d < r2 * r2) {
			Collision result;
			result.normal = normal / std::sqrt(d);
			result.contacts[0] = p0.to_local_vector(result.normal * p0.dims[0]);
			result.contacts[1] = p1.to_local_vector(result.normal * p0.dims[1]);
			
			return result;
		}
		else {
			return std::nullopt;
		}
	}
}