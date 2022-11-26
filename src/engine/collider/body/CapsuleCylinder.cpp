#include <pbd/engine/collider/BodyBodyCollision.hpp>

#include <pbd/engine/RigidBody.hpp>

namespace pbd {
	std::optional<Collision> capsule_cylinder_collide(const RigidBody& p0, const RigidBody& p1) {
		// Perform collision exactly like the capsule-capsule collision,
		// but check to see if then end caps of the cylinder are involved.

		// If the end caps are involved, project the capsule point onto the end cap plane, limit to the end cap ring.
		

		return std::nullopt;
	}
}