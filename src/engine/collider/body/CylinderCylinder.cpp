#include <pbd/engine/collider/BodyBodyCollision.hpp>

#include <pbd/engine/RigidBody.hpp>

namespace pbd {
	// Much harder than capsule for some reason.
	std::optional<Collision> cylinder_cylinder_collide(const RigidBody& p0, const RigidBody& p1) {
		// Calculate the shortest segment between the two cylinders.
		// If the points are past the end, clamp to the endpoints.
		
		// There are three possibilities:
		// Both endpoints are clamped.
		// One endpoint is clamped.
		// Neither are clamped.

		// When neither are clamped, we have no more work to do.
		// When one is clamped, we have to find the nearest point on the endcap to the other, then correct both points.
		// When both are clamped, we just have to project the axis between them onto each endcap to find the closest points.
		  
		// Procedure for end cap corrections:
		//	Project p0 onto other endcap.
		//  Move projected point to ring of end cap (if not possible, axis is aligned, early exit edge case)
		//  Project ring point onto the first line.
		//  offset by original normal * radius
		//  Project onto other end cap again, clamp to end cap radius.
		// Fin.

		return std::nullopt;
	}
}