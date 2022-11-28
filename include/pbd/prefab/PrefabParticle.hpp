#pragma once
#include <cinttypes>
#include <string>
#include <pbd/common/Types.hpp>

namespace pbd {
	// Particle for a prefab.
	class PrefabParticle {
	public:
		static void serialize(const PrefabParticle & particle, std::string& output);
		static const char* deserialize(const char* first, const char* last, PrefabParticle& particle);

		PrefabParticle();
		PrefabParticle(const vec3_t & _pos, real_t _imass, real_t _radius, uint32_t _cgroups, uint32_t _cmask);
		PrefabParticle(const vec3_t& _pos, const vec3_t& _vel, real_t _imass, real_t _radius, uint32_t _cgroups, uint32_t _cmask);
		
		vec3_t position, velocity;
		real_t imass, radius;
		uint32_t collision_groups, collision_mask;
	};
}