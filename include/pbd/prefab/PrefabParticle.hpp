#pragma once
#include <cinttypes>
#include <string>
#include <glm/vec3.hpp>

namespace pbd {
	// Particle for a prefab.
	class PrefabParticle {
	public:
		static void serialize(const PrefabParticle & particle, std::string& output);
		static const char* deserialize(const char* first, const char* last, PrefabParticle& particle);

		PrefabParticle();
		PrefabParticle(const glm::vec3 & _pos, float _imass, float _radius, uint32_t _flags);
		PrefabParticle(const glm::vec3& _pos, const glm::vec3& _vel, float _imass, float _radius, uint32_t _flags);
		
		glm::vec3 position, velocity;
		float imass, radius;
		uint32_t flags;
	};
}