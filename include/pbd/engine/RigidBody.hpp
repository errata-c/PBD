#pragma once
#include <cinttypes>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	glm::vec3 rotate(const glm::quat & rot, const glm::vec3& vec);
	glm::vec3 reverse_rotate(const glm::quat& rot, const glm::vec3& vec);

	enum class Shape {
		Capsule,
		OBB,
		Cylinder,
		Sphere
	};

	/*
	This class overlaps completely with the Particle class, 
	with the following data members being in the same order and size as Particle:
		collision_groups, collision_mask, position, velocity, imass, and the first element of dims (radius)
	*/
	class RigidBody {
	public:	
		uint32_t collision_groups, collision_mask;

		glm::vec3 position, velocity;

		float imass;

		// Sphere uses first dim as radius
		// Cylinder and Capsule use first as radius, and second as height
		// OBB uses dims as the half extents of the box
		glm::vec3 dims;

		glm::quat orientation;
		glm::vec3 angular_velocity;

		// This is the inverse of the inertial tensor!
		glm::vec3 inertia;
		
		Shape shape;


		float& width() noexcept;
		const float& width() const noexcept;
		float& radius() noexcept;
		const float& radius() const noexcept;
		float& height() noexcept;
		const float& height() const noexcept;
		float& depth() noexcept;
		const float& depth() const noexcept;

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