#include <pbd/collide/Particle.hpp>
#include <pbd/Engine.hpp>

#include <pbd/constraint/Utils.hpp>

namespace pbd {
	void CollideParticle::eval(Engine& engine, float rdt2) const {
		float w0 = engine.particle.invMass[p0];
		float w1 = engine.particle.invMass[p1];

		float w = w0 + w1;
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// references, we are going to modify these in place.
		glm::vec3& x0 = engine.particle.pos[p0];
		glm::vec3& x1 = engine.particle.pos[p1];

		// Grads 4x3
		glm::vec3 grad = x0 - x1;
		float length = glm::length(grad);
		float C = length - distance;
		if (C >= 0.f || length < 1e-5f) {
			// Do nothing when the constraint is above zero
			// Also prevent divide by zero
			return;
		}

		grad /= length;

		float lambda = C / w;

		// Update the positions for the next constraint to use.
		x0 += -lambda * w0 * grad;
		x1 +=  lambda * w1 * grad;

		// Frictional delta
		// [(x0 - prev x0) - (x1 - prev x1)] perpendicular to normal (x0 - x1, see grad above)

		// Current positional deltas for the substep.
		glm::vec3 px0 = x0 - engine.particle.prevPos[p0];
		glm::vec3 px1 = x1 - engine.particle.prevPos[p1];

		// Friction normal is the grad
		px0 = perpendicular(px0, grad);
		px1 = perpendicular(px1, grad);

		px0 = frictionDelta(px0, -C, engine.staticFriction, engine.kineticFriction);
		px1 = frictionDelta(px1, -C, engine.staticFriction, engine.kineticFriction);

		x0 += w0 * w * px0;
		x1 += w1 * w * px1;
	}
}