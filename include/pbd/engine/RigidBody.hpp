#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	enum class Shape {
		Capsule,
		OBB,
		Cylinder,
		Sphere
	};

	class RigidBody {
	public:
		Shape shape;

		glm::vec3 position, velocity;

		float imass;

		uint32_t collision_groups, collision_mask;

		glm::quat orientation;
		glm::vec3 angular_velocity;

		// This is the inverse of the inertial tensor!
		glm::vec3 inertia;

		// Sphere uses first dim as radius
		// Cylinder and Capsule use first as radius, and second as height
		// OBB uses dims as the half extents of the box
		glm::vec3 dims;

		BBox3 bounds() const noexcept;
		BBox3 skewed_bounds(float delta) const noexcept;
		void calculate_inertia();

		glm::vec3 to_local_vector(const glm::vec3& v) const noexcept;
		glm::vec3 to_world_vector(const glm::vec3& v) const noexcept;
		glm::vec3 to_local(const glm::vec3& v) const noexcept;
		glm::vec3 to_world(const glm::vec3& v) const noexcept;

		Transform3 transform() const noexcept;
	};
}