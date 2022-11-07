#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>

#include <pbd/common/Types.hpp>
#include <pbd/engine/ConstraintList.hpp>
#include <pbd/engine/ParticleList.hpp>
#include <pbd/engine/BodyList.hpp>

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


		struct Particles {
			// The particles data and their execution data
			ParticleList list;

			// The forces and previous positions of the particles
			std::vector<glm::vec3> forces, prevPos;
		} particles;
		
		struct Bodies {
			BodyList list;

			std::vector<glm::vec3> forces, prevPos;
			std::vector<glm::vec3> torques, prevOrientation;

		} bodies;

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

		void clear_forces();
		void save_positions();
	};
}