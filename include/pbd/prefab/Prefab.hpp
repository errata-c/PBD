#pragma once
#include <string>
#include <vector>
#include <array>
#include <cinttypes>

#include <glm/vec3.hpp>

#include <pbd/engine/ConstraintList.hpp>
#include <pbd/prefab/PrefabParticle.hpp>
#include <pbd/prefab/PrefabTracker.hpp>

namespace pbd {
	/*
	This class stores a number of particles and constraints for them.
	At runtime the object can then be instanced into the scene as needed.

	Additionally, can store a list of bones to extract rotations from
	*/
	class Prefab {
	public:
		enum class Error {
			None,
			TrackerIndexRange,
			ConstraintIndexRange,
			ParticleMassDomain,
			ParticleRadiusDomain,
			NoParticles
		};

		static void serialize(const Prefab& prefab, std::string& output);
		static const char* deserialize(const char* first, const char* last, Prefab& prefab);

		Error validate() const noexcept;

		bool empty() const noexcept;
		void clear();

		size_t num_particles() const noexcept;
		size_t num_constraints() const noexcept;
		size_t num_trackers() const noexcept;



		std::vector<PrefabParticle> particles;
		std::vector<PrefabTracker> trackers;
		ConstraintList constraints;
	};
}