#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace pbd {
	enum class Shape {
		Capsule,
		OBB,
		Cylinder,
		Sphere
	};

	class RigidBody {
	public:
		glm::vec3 position, velocity;
		float imass;

		uint32_t collision_groups, collision_mask;

		glm::quat orientation;
		glm::vec3 angular_velocity;
		glm::vec3 inertial_tensor;
	};
}