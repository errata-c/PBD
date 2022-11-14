#include <pbd/engine/collider/BodyBodyCollision.hpp>
#include <pbd/engine/collider/BodyParticleCollision.hpp>

#include <pbd/engine/RigidBody.hpp>

namespace pbd {
	std::optional<Collision> cylinder_sphere_collide(const RigidBody& p0, const RigidBody& p1) {
		PointNorm pn = sdCylinder(p0.to_local(p1.position), p0.radius(), p0.height());
		
		if (pn.distance < p1.radius()) {
			Collision result;
			result.normal = p0.to_world_vector(pn.normal);
			result.contacts[0] = pn.point;
			result.contacts[1] = p1.to_local_vector(-result.normal * p1.radius());

			return result;
		}
		else {
			return std::nullopt;
		}
	}
}