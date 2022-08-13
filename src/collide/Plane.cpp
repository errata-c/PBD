#include <pbd/collide/Plane.hpp>
#include <pbd/Engine.hpp>

#include <pbd/constraint/Utils.hpp>


namespace pbd {
	void CollidePlane::eval(Engine& engine, float rdt2) const {
		// C(x0) = dot((x0 - origin), normal) - x0.radius >= 0
		// grad(C wrt x0) = normal

		// >= 0 constraint means we only project the particle when C(x0) < 0

		glm::vec3& x0 = engine.particle.pos[id];
		float w0 = engine.particle.invMass[id];
		if (w0 < 1e-5f) {
			// inverse mass of zero means the particle is infinitely massive, will not move.
			return;
		}

		// Calculate the constraint value
		float C = glm::dot(x0 - origin, normal) - engine.particle.radius[id];
		if (C >= 0.f) {
			// Constraint does not apply.
			return;
		}

		// w0 / (w0 + 0) == 1
		float lambda = -C;

		x0 += lambda * normal;

		// Calculate interpenetration distance, ie overlap distance.
		float d = -C;

		// Mass not needed for a static plane collision.

		// The total delta over step
		glm::vec3 pdelta = x0 - engine.particle.prevPos[id];

		// Find the perpendicular element of that motion.
		glm::vec3 perp = perpendicular(pdelta, normal);

		x0 += frictionDelta(perp, d, engine.staticFriction, engine.kineticFriction);
	}

	void CollidePlane::remap(int32_t offset) {
		id += offset;
	}
}