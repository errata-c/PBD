#include <pbd/engine/Engine.hpp>
#include <algorithm>
#include <limits>

namespace pbd {
	Engine::Engine()
		: static_friction(1.f)
		, kinetic_friction(1.f)
		, gravity(0,-9.8,0)
		, substeps(4)
		, dt(1.0 / 60.0)
	{}

	size_t Engine::num_particles() const noexcept {
		return particles.size();
	}
	size_t Engine::num_constraints() const noexcept {
		return constraints.size();
	}

	// Run one iteration of the solver
	void Engine::solve() {
		// Collision constraints would be generated here!
		// If needed we can predict where the positions will be roughly, then find the collisions.

		
		// Run the iteration substeps.
		float sdt = dt / float(substeps);
		for (int i = 0; i < substeps; ++i) {
			predictPositions(sdt);
			applyConstraints(sdt);
			updateParticles(sdt);
		}

		// Clear the external forces
		for (glm::vec3 & force : particles.force) {
			force = glm::vec3(0.f);
		}
	}

	void Engine::predictPositions(float sdt) {
		// Store these starting positions for comparison in the constraints.
		particles.prevPos.assign(particles.pos.begin(), particles.pos.end());

		for (int64_t i = 0, count = num_particles(); i < count; ++i) {
			float imass = particles.invMass[i];
			if (imass < 1e-5f) {
				continue;
			}

			glm::vec3 & velocity = particles.velocity[i];
			glm::vec3 & position = particles.pos[i];

			// External forces
			const glm::vec3 & force = particles.force[i];
			
			// Simple euler integration
			velocity = velocity + sdt * (gravity + force * imass);
			position = position + sdt * velocity;
		}
	}
	void Engine::applyConstraints(float sdt) {
		// Run all the constraints

		float rdt2 = 1.f / (sdt * sdt);
		for (int64_t i = 0, count = constraints.size(); i < count; ++i) {
			constraints[i].eval(*this, rdt2);
		}
	}
	void Engine::updateParticles(float sdt) {
		sdt = 1.f / sdt;

		for (int64_t i = 0, count = num_particles(); i < count; ++i) {
			particles.velocity[i] = (particles.pos[i] - particles.prevPos[i]) * sdt;
		}
	}
}