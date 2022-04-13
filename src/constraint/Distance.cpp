#include <pbd/constraint/Distance.hpp>
#include <pbd/Engine3D.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void Distance3D::eval(Engine3D& engine) const {
		float w0 = engine.invMass[p0];
		float w1 = engine.invMass[p1];

		float w = w0 + w1;
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// references, we are going to modify these in place.
		glm::vec3& x0 = engine.pos[p0];
		glm::vec3& x1 = engine.pos[p1];

		// Grads 4x3
		glm::vec3 grad = x0 - x1;
		float length = glm::length(grad);
		if (length <= 1e-5f) {
			// We have undefined gradients here. We can't do anything.
			return;
		}
		
		grad /= length;

		// The lambda value determines how the movement is to be weighted.
		float lambda = -(length - initialLength) / w;

		// Update the positions for the next constraint to use.
		x0 += lambda * w0 * grad;
		x1 += -lambda * w1 * grad;
	}
}