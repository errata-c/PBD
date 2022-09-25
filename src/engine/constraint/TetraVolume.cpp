#include <pbd/engine/constraint/TetraVolume.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void ConstraintTetraVolume::eval(Engine & engine, float rdt2) const {
		std::array<glm::vec3*, 4> x;
		// Fetch the addresses of the positions
		for (int i = 0; i < ids.size(); ++i) {
			x[i] = &engine.particle.pos[ids[i]];
		}

		std::array<glm::vec3, 4> grads;

		float w = 0;
		for (int i = 0; i < 4; ++i) {
			glm::vec3 t0 = *x[faceOrder[i][1]] - *x[faceOrder[i][0]];
			glm::vec3 t1 = *x[faceOrder[i][2]] - *x[faceOrder[i][0]];

			grads[i] = glm::cross(t0, t1);
			grads[i] *= (1.0 / 6.0);

			w += engine.particle.invMass[ids[i]] * glm::dot(grads[i], grads[i]);
		}

		w += compliance * rdt2;
		
		if (w <= 1e-5f) {
			// Avoid division by zero
			return;
		}

		// Calculate the volume
		float currentVolume = 0;
		{
			glm::vec3 t0 = *x[1] - *x[0];
			glm::vec3 t1 = *x[2] - *x[0];
			glm::vec3 t2 = *x[3] - *x[0];

			glm::vec3 t3 = glm::cross(t0, t1);
			currentVolume = glm::dot(t3, t2) / 6.0;
		}

		// Apply the deltas
		float lambda = -(currentVolume - volume) / w;
		for (int i = 0; i < 4; ++i) {
			*x[i] += lambda * engine.particle.invMass[ids[i]] * grads[i];
		}
	}

	void ConstraintTetraVolume::remap(int32_t offset) {
		for (int32_t& id : ids) {
			id += offset;
		}
	}
	void ConstraintTetraVolume::transform(const Transform3& form) {
		volume *= form.size;
	}
}