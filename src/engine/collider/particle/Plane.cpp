#include <pbd/engine/collider/particle/Plane.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/common/Utils.hpp>

namespace pbd {
	void CollidePlane::eval(Engine& engine, real_t rdt2) const {
		Particle & part = engine.particles.list[id];

		// C(x0) = dot((x0 - origin), normal) - x0.radius >= 0
		// grad(C wrt x0) = normal

		// >= 0 constraint means we only project the particle when C(x0) < 0

		vec3_t& x0 = part.position;
		real_t w0 = part.imass;
		if (w0 < 1e-5f) {
			// inverse mass of zero means the particle is infinitely massive, will not move.
			return;
		}

		// Calculate the constraint value
		real_t C = glm::dot(x0 - origin, normal) - part.radius;
		if (C >= 0.f) {
			// Constraint does not apply.
			return;
		}

		// w0 / (w0 + 0) == 1
		real_t lambda = -C;

		x0 += lambda * normal;

		// Calculate interpenetration distance, ie overlap distance.
		real_t d = -C;

		// Mass not needed for a static plane collision.

		// The total delta over step
		vec3_t pdelta = x0 - engine.particles.prevPos[id];

		// Find the perpendicular element of that motion.
		vec3_t perp = perpendicular(pdelta, normal);

		x0 += friction_delta(perp, d, engine.static_friction, engine.kinetic_friction);
	}

	void CollidePlane::remap(int32_t offset) {
		id += offset;
	}
}