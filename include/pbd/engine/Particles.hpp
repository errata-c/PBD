#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class PrefabParticle;

	class Particles {
	public:
		~Particles() = default;
		Particles() = default;
		Particles(Particles&&) noexcept = default;
		Particles& operator=(Particles&&) noexcept = default;
		Particles(const Particles&) = default;
		Particles& operator=(const Particles& ) = default;

		size_t size() const noexcept;
		bool empty() const noexcept;

		int32_t add(const glm::vec3& position, const glm::vec3& velocity, float invMass, float radius, uint32_t _flags = 0u);
		int32_t add(const glm::vec3& position, float invMass, float radius, uint32_t _flags = 0u);
		int32_t add(const PrefabParticle& particle);
		int32_t add(const PrefabParticle& particle, const Transform3& form);

		void shift(int32_t first, int32_t last, int32_t amount);
		void pop(int32_t amount);
		void clear();
		void reserve(int32_t amount);

		std::vector<glm::vec3> pos, prevPos, velocity, force;
		std::vector<float> invMass, radius;
		std::vector<uint32_t> flags;
	};
}