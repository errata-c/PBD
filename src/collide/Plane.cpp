#include <pbd/collide/Plane.hpp>
#include <pbd/Engine.hpp>

#include <glm/geometric.hpp>
#include <glm/gtx/projection.hpp>

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
		float c = glm::dot(x0 - origin, normal) - engine.particle.radius[id];
		if (c >= 0.f) {
			// Constraint does not apply.
			return;
		}

		// w0 / (w0 + 0) == 1
		float lambda = -c;

		x0 += lambda * normal;

		// Calculate interpenetration distance, ie overlap distance.
		float d = -c;

		// Mass not needed for a static plane collision.

		// The total delta over substeps
		glm::vec3 pdelta = x0 - engine.particle.prevPos[id];

		// Find the perpendicular element of that motion.
		glm::vec3 perp = pdelta - normal * glm::dot(pdelta, normal);
		float plen = glm::length(perp);

		if (plen < (engine.staticFriction * d)) {
			// Completely eliminate tangential motion
			x0 -= perp;
		}
		else {
			// Damp tangential motion
			x0 -= perp * std::min((engine.kineticFriction * d) / plen, 1.f);
		}
	}
}