#include <pbd/engine/constraint/particle/Distance.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void CDistance::eval(Engine& engine, float rdt2) const {
		Particle& part0 = engine.particles.list[p0];
		Particle& part1 = engine.particles.list[p1];

		float w0 = part0.imass;
		float w1 = part1.imass;

		float w = w0 + w1 + (compliance * rdt2);
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// references, we are going to modify these in place.
		glm::vec3& x0 = part0.position;
		glm::vec3& x1 = part1.position;
		
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

	void CDistance::remap(int32_t offset) {
		p0 += offset;
		p1 += offset;
	}
	void CDistance::transform(const Transform3& form) {
		length *= form.size;
	}
}