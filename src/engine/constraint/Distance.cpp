#include <pbd/engine/constraint/Distance.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void ConstraintDistance::eval(Engine& engine, float rdt2) const {
		float w0 = engine.particle.invMass[p0];
		float w1 = engine.particle.invMass[p1];

		float w = w0 + w1 + (compliance * rdt2);
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// references, we are going to modify these in place.
		glm::vec3& x0 = engine.particle.pos[p0];
		glm::vec3& x1 = engine.particle.pos[p1];

		
		glm::vec3 grad = x0 - x1;
		float gradLen = glm::length(grad);
		if (gradLen <= 1e-5f) {
			// Zero length gradient means zero length delta. No further work needed.
			return;
		}
		
		grad /= gradLen;

		// The lambda value determines how the movement is to be weighted.
		// -C / (w1*|grad(C0)| + w2*|grad(C1)| + compliance / dt^2)
		float C = (gradLen - length);
		float lambda = C / w;

		// Update the positions for the next constraint to use.
		x0 += -lambda * w0 * grad;
		x1 += lambda * w1 * grad;
	}

	void ConstraintDistance::remap(int32_t offset) {
		p0 += offset;
		p1 += offset;
	}
}