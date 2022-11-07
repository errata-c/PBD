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
		return particles.list.size();
	}
	size_t Engine::num_constraints() const noexcept {
		return constraints.size();
	}

	// Run one iteration of the solver
	void Engine::solve() {
		particles.forces.resize(particles.list.size(), glm::vec3(0));
		
		// Collision constraints would be generated here!
		// If needed we can predict where the positions will be roughly, then find the collisions.

		
		// Run the iteration substeps.
		float sdt = dt / float(substeps);
		for (int i = 0; i < substeps; ++i) {
			predictPositions(sdt);
			applyConstraints(sdt);
			updateParticles(sdt);
		}

		clear_forces();
	}

	void Engine::predictPositions(float sdt) {
		// Store these starting positions for comparison in the constraints.
		save_positions();

		for (int64_t i = 0, count = num_particles(); i < count; ++i) {
			float imass = particles.list[i].imass;
			if (imass < 1e-5f) {
				continue;
			}

			glm::vec3 & velocity = particles.list[i].velocity;
			glm::vec3 & position = particles.list[i].position;

			// External forces
			const glm::vec3 & force = particles.forces[i];
			
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

		auto pit = particles.prevPos.begin();
		for (Particle & particle: particles.list) {
			particle.velocity = (particle.position - *pit) * sdt;
			++pit;
		}
	}


	void Engine::clear_forces() {
		std::fill(particles.forces.begin(), particles.forces.end(), glm::vec3(0.f));
	}
	void Engine::save_positions() {
		auto it = particles.prevPos.begin();
		for (const Particle& particle: particles.list) {
			*it = particle.position;
			++it;
		}
	}
}