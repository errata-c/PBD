#include <pbd/engine/particle/constraint/TetraVolume.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

namespace pbd {
	void ConstraintTetraVolume::eval(Engine & engine, float rdt2) const {
		std::array<Particle*, 4> p;
		// Fetch the addresses of the positions
		for (int i = 0; i < ids.size(); ++i) {
			p[i] = &engine.particles[ids[i]];
		}

		std::array<glm::vec3, 4> grads;

		float w = 0;
		for (int i = 0; i < 4; ++i) {
			glm::vec3 t0 = p[faceOrder[i][1]]->position - p[faceOrder[i][0]]->position;
			glm::vec3 t1 = p[faceOrder[i][2]]->position - p[faceOrder[i][0]]->position;

			grads[i] = glm::cross(t0, t1);
			grads[i] *= (1.0 / 6.0);

			w += p[i]->imass * glm::dot(grads[i], grads[i]);
		}

		w += compliance * rdt2;
		
		if (w <= 1e-5f) {
			// Avoid division by zero
			return;
		}

		// Calculate the volume
		float currentVolume = 0;
		{
			glm::vec3 t0 = p[1]->position - p[0]->position;
			glm::vec3 t1 = p[2]->position - p[0]->position;
			glm::vec3 t2 = p[3]->position - p[0]->position;

			glm::vec3 t3 = glm::cross(t0, t1);
			currentVolume = glm::dot(t3, t2) / 6.0;
		}

		// Apply the deltas
		float lambda = -(currentVolume - volume) / w;
		for (int i = 0; i < 4; ++i) {
			p[i]->position += lambda * p[i]->imass * grads[i];
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