#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>

namespace pbd {
	// Particle data for runtime.
	class Particle {
	public:
		uint32_t collision_groups, collision_mask;

		vec3_t position, velocity;
		real_t imass, radius;
	};
}