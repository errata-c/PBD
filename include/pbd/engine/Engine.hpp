#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>

#include <pbd/common/Types.hpp>
#include <pbd/engine/ConstraintList.hpp>
#include <pbd/engine/ParticleList.hpp>

namespace pbd {
	class Engine {
	public:
		Engine();

		glm::vec3 gravity;
		float dt;
		int substeps;
		// Perhaps change the global friction to be a particle specific thing?
		// The issue is that friction is calculated on a material pairing basis.
		float kinetic_friction;
		float static_friction;

		// The particles, and their execution data
		ParticleList particles;
		// The constraints and their execution data
		ConstraintList constraints;

		//void reserve(int64_t count);
		size_t num_particles() const noexcept;
		size_t num_constraints() const noexcept;

		void solve();
	private:
		void predictPositions(float sdt);
		void applyConstraints(float sdt);
		void updateParticles(float sdt);
	};
}