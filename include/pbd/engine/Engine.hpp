#pragma once
#include <vector>
#include <cinttypes>

#include <pbd/common/Types.hpp>
#include <pbd/engine/constraint/ConstraintList.hpp>
#include <pbd/engine/ParticleList.hpp>
#include <pbd/engine/BodyList.hpp>
#include <pbd/common/BBox.hpp>

namespace pbd {
	class Engine {
	public:
		Engine();

		BBox3 world_bounds;

		vec3_t gravity;
		real_t dt;
		int substeps;

		// Perhaps change the global friction to be a particle specific thing?
		// The issue is that friction is calculated on a material pairing basis.
		real_t kinetic_friction;
		real_t static_friction;

		struct Particles {
			// The particles data and their execution data
			ParticleList list;

			// The forces and previous positions of the particles
			std::vector<vec3_t> forces, prevPos;
		} particles;
		
		struct Bodies {
			BodyList list;

			std::vector<vec3_t> forces, torques, prevPos;
			std::vector<quat_t> prevOrientation;

		} bodies;

		// The constraints and their execution data
		ConstraintList constraints;

		//void reserve(int64_t count);
		size_t num_particles() const noexcept;
		size_t num_bodies() const noexcept;
		size_t num_constraints() const noexcept;

		void solve();
	private:
		void prepare_round();

		void integrate_changes(real_t sdt);
		void apply_constraints(real_t sdt);
		void update_states(real_t sdt);

		void clear_forces();
		void save_previous();
	};
}