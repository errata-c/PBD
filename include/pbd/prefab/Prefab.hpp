#pragma once
#include <string>
#include <vector>
#include <array>
#include <cinttypes>

#include <glm/vec3.hpp>

#include <pbd/engine/constraint/ConstraintList.hpp>
#include <pbd/prefab/PrefabParticle.hpp>
#include <pbd/prefab/PrefabTracker.hpp>

namespace pbd {
	/*
	This class stores a number of particles and constraints for them.
	At runtime the object can then be instanced into the scene as needed.

	Additionally, can store a list of bones to extract rotations from.
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

		int32_t add_particle(const vec3_t& _pos, real_t _imass, real_t _radius, uint32_t _groups, uint32_t _mask);
		int32_t add_particle(const vec3_t& _pos, const vec3_t& _vel, real_t _imass, real_t _radius, uint32_t _groups, uint32_t _mask);

		void add_tracker(std::string_view _name, int32_t p0, int32_t p1, int32_t p2, int32_t p3);

		template<typename T>
		int64_t add_constraint(const T& cval) {
			return constraints.add(cval);
		}

		std::vector<PrefabParticle> particles;
		std::vector<PrefabTracker> trackers;
		ConstraintList constraints;
	};
}