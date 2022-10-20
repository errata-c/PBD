#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class PrefabParticle;

	class ParticleList {
	public:
		~ParticleList() = default;
		ParticleList() = default;
		ParticleList(ParticleList&&) noexcept = default;
		ParticleList& operator=(ParticleList&&) noexcept = default;
		ParticleList(const ParticleList&) = default;
		ParticleList& operator=(const ParticleList& ) = default;

		size_t size() const noexcept;
		bool empty() const noexcept;

		int32_t add(const glm::vec3& position, const glm::vec3& velocity, float invMass, float radius, uint32_t _group = 0u, uint32_t _mask = 0u);
		int32_t add(const glm::vec3& position, float invMass, float radius, uint32_t _group = 0u, uint32_t _mask = 0u);
		int32_t add(const PrefabParticle& particle);
		int32_t add(const PrefabParticle& particle, const Transform3& form);

		void shift(int32_t first, int32_t last, int32_t amount);
		void pop(int32_t amount);
		void clear();
		void reserve(int32_t amount);

		// Force should be kept separate, it is only accessed in a special pass. 

		// Position and invMass are needed for all the constraints.

		// Previous position and radius are needed for all the particle collision constraints

		// Collision data is only needed during the setup phase.
		struct CollisionData {
			uint32_t groups, mask;
		};

		std::vector<glm::vec3> pos, prevPos, velocity, force;
		std::vector<float> invMass, radius;
		std::vector<CollisionData> collision;
	};
}