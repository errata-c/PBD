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

		float lambda = -c * w0;

		// Update the position
		x0 += lambda * normal;

		// Calculate interpenetration distance, ie overlap distance.
		float d = -c;

		// Mass not needed for a static plane collision.

		// The total delta over substeps
		glm::vec3 pdelta = x0 - engine.particle.prevPos[id];

		// Project the particle delta so that its perpendicular to the contact normal.
		glm::vec3 perp = glm::proj(pdelta, normal);
		float plen = glm::length(perp);

		if (plen < (engine.staticFriction * d)) {
			x0 += perp;	
		}
		else {
			x0 += perp * std::min((engine.kineticFriction * d) / plen, 1.f);
		}
	}
}