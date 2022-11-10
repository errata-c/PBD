#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>

namespace pbd {
	// Particle data for runtime.
	class Particle {
	public:
		glm::vec3 position, velocity;
		float imass, radius;

		uint32_t collision_groups, collision_mask;
	};
}