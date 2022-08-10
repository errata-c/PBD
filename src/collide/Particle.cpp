#include <pbd/collide/Particle.hpp>
#include <pbd/Engine.hpp>

#include <glm/geometric.hpp>

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
		if ((length > distance) || (length < 1e-5f)) {
			// Do nothing when the constraint is above zero
			// Also prevent divide by zero
			return;
		}

		grad /= length;

		// The lambda value determines how the movement is to be weighted.
		float lambda = -(length - distance) / w;

		// Update the positions for the next constraint to use.
		x0 += lambda * w0 * grad;
		x1 += -lambda * w1 * grad;

		// Frictional delta
		// [(x0 delta) - (x1 delta)] perpendicular to normal (x0 - x1, see grad above)

	}
}