#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace pbd {
	struct RigidBody {
		glm::vec3 position, velocity;
		float imass;

		uint32_t collision_groups, collision_mask;
	};
}