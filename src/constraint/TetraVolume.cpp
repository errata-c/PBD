#include <pbd/constraint/TetraVolume.hpp>
#include <pbd/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void TetraVolume::eval(Engine & engine) const {
		std::array<glm::vec3*, 4> x;
		// Fetch the addresses of the positions
		for (int i = 0; i < ids.size(); ++i) {
			x[i] = &engine.pos[ids[i]];
		}

		std::array<glm::vec3, 4> grads;

		float w = 0;
		for (int i = 0; i < 4; ++i) {
			glm::vec3 t0 = *x[faceOrder[i][1]] - *x[faceOrder[i][0]];
			glm::vec3 t1 = *x[faceOrder[i][2]] - *x[faceOrder[i][0]];

			grads[i] = glm::cross(t0, t1);
			grads[i] *= (1.0 / 6.0);

			w += engine.invMass[ids[i]] * glm::dot(grads[i], grads[i]);
		}
	
		if (w <= 1e-5f) {
			// Avoid division by zero
			return;
		}

		// Calculate the volume
		float volume = 0;
		{
			glm::vec3 t0 = *x[1] - *x[0];
			glm::vec3 t1 = *x[2] - *x[0];
			glm::vec3 t2 = *x[3] - *x[0];

			glm::vec3 t3 = glm::cross(t0, t1);
			volume = glm::dot(t3, t2) / 6.0;
		}

		// Apply the deltas
		float lambda = -(volume - initialVolume) / w; // (w + a) for compliance calculation
		for (int i = 0; i < 4; ++i) {
			*x[i] += lambda * engine.invMass[ids[i]] * grads[i];
		}
	}
}