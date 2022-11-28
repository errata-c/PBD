#include <pbd/engine/collider/BodyBodyCollision.hpp>
#include <pbd/engine/collider/BodyParticleCollision.hpp>

#include <pbd/engine/RigidBody.hpp>
#include <pbd/common/Utils.hpp>
#include <glm/vec2.hpp>

namespace pbd {
	// b0 and b1 are reversed in this function signature ON PURPOSE
	// Do not change it.
	std::optional<Collision> capsule_obb_collide(const RigidBody& b1, const RigidBody& b0) {
		// Work relative to the obb

		/*
		Three major cases:
			The capsule is axis aligned with the OBB
			The capsule is clamped against an edge
			The capsule contacts an edge
		*/

		quat_t orient = b1.orientation * glm::conjugate(b0.orientation);
		vec3_t origin = rotate(glm::conjugate(b0.orientation), b1.position - b0.position);

		vec3_t axis = rotate(orient, vec3_t(0.f, 0.f, 1.f));

		// Project the origin point onto the capsule line segment.
		real_t t = -glm::dot(origin, axis);

		// If projection distance is greater than OBB max sphere, no collision
		// Note: using pythagorean theorem here ('t' is wrong side of triangle)
		if ((glm::dot(origin, origin) - t * t) > glm::dot(b0.dims, b0.dims)) {
			return std::nullopt;
		}
		
		// If the projection needs to be clamped to capsule:
		// (Assuming that dims are always positive values!)
		if (std::abs(t) > b1.dims.y) {
			// Devolves into sphere OBB intersection
			vec3_t tmp = origin + axis * glm::clamp(t, -b1.dims.y, b1.dims.y);
			PointNorm pn = sdBox(tmp, b0.dims);
			if (pn.distance < b1.radius()) {
				Collision result;
				result.normal = pn.normal;
				result.contacts[0] = pn.point;
				result.contacts[1] = b1.to_local_vector(-result.normal * b1.radius());

				return result;
			}
			else {
				return std::nullopt;
			}
		}
		else {
			// Does not need to be clamped (yet)
			// Find the face of the OBB the direction points to.


		}
		
		
		
		//
		// Else:
		//	 The projection direction forms a plane with the axis of the capsule.
		//   Find the face of the OBB the direction points to.
		//   If perfectly aligned with face normal, just project onto face directly, FIN
		//   Otherwise, find the edge of the face the direction favors.
		//   
		//   Find the intersection of that edge with the plane formed earlier
		//   That intersection point is the OBB contact point.
		//   Project that contact point back onto the capsule
		// 
		//   If the projection needs to be clamped:
		//		Find the axis of penetration
		//		Project OBB contact point to surface of OBB
		//		Project Capsule contact point to opposite surface of capsule end sphere
		//   Else:
		//      The contact points are already found. Just finish up and return the result.
		//		FIN
		
		return std::nullopt;
	}
}