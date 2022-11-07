#include <pbd/engine/particle/collide/Particle.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/common/Utils.hpp>

namespace pbd {
	void CollideParticle::eval(Engine& engine, float rdt2) const {
		std::array<Particle*, 2> p{
			&engine.particles.list[ids[0]],
			&engine.particles.list[ids[1]]
		};

		float w0 = p[0]->imass;
		float w1 = p[1]->imass;

		float w = w0 + w1;
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// Grads 4x3
		glm::vec3 grad = p[0]->position - p[1]->position;
		float length = glm::length(grad);
		float distance = p[0]->radius + p[1]->radius;
		float C = length - distance;
		if (C >= 0.f || length < 1e-5f) {
			// Do nothing when the constraint is above zero
			// Also prevent divide by zero
			return;
		}

		grad /= length;

		float lambda = C / w;

		// Update the positions for the next constraint to use.
		p[0]->position += -lambda * w0 * grad;
		p[1]->position +=  lambda * w1 * grad;

		// Frictional delta
		// [(x0 - prev x0) - (x1 - prev x1)] perpendicular to normal (x0 - x1, see grad above)

		// Current positional deltas for the substep.
		glm::vec3 px0 = p[0]->position - engine.particles.prevPos[ids[0]];
		glm::vec3 px1 = p[1]->position - engine.particles.prevPos[ids[1]];

		// Friction normal is the grad
		px0 = perpendicular(px0, grad);
		px1 = perpendicular(px1, grad);

		px0 = friction_delta(px0, -C, engine.static_friction, engine.kinetic_friction);
		px1 = friction_delta(px1, -C, engine.static_friction, engine.kinetic_friction);

		p[0]->position += w0 * w * px0;
		p[1]->position += w1 * w * px1;
	}

	void CollideParticle::remap(int32_t offset) {
		ids[0] += offset;
		ids[1] += offset;
	}
}