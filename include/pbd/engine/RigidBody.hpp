#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>

namespace pbd {

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

		vec3_t position, velocity;

		real_t imass;

		// Sphere uses first dim as radius
		// Cylinder and Capsule use first as radius, and second as height
		// OBB uses dims as the half extents of the box
		vec3_t dims;

		quat_t orientation;
		vec3_t angular_velocity;

		// reciprocal of the inertial tensor (simplified to a 3 component vector)

		vec3_t inverse_inertia;
		
		Shape shape;

		RigidBody();
		RigidBody(
			Shape shape, 
			const Transform3& form, 
			real_t imass, 
			const vec3_t& dims, 
			uint32_t cgroups = 0, 
			uint32_t cmask = 0
		);
		RigidBody(
			Shape shape,
			const vec3_t& position,
			const quat_t& orientation,
			real_t imass,
			const vec3_t& dims,
			uint32_t cgroups = 0,
			uint32_t cmask = 0
		);
		RigidBody(
			Shape shape, 
			const Transform3& form, 
			real_t imass, 
			const vec3_t& dims, 
			const vec3_t& velocity, 
			const vec3_t& angular_velocity,
			uint32_t cgroups = 0,
			uint32_t cmask = 0
		);
		RigidBody(
			Shape shape,
			const vec3_t& position,
			const quat_t& orientation,
			real_t imass,
			const vec3_t& dims,
			const vec3_t& velocity,
			const vec3_t& angular_velocity,
			uint32_t cgroups = 0,
			uint32_t cmask = 0
		);

		// Update the mass value, also recalculates the inertial tensor
		void set_mass(real_t mass) noexcept;
		void set_imass(real_t _imass) noexcept;

		real_t& width() noexcept;
		const real_t& width() const noexcept;
		real_t& radius() noexcept;
		const real_t& radius() const noexcept;
		real_t& height() noexcept;
		const real_t& height() const noexcept;
		real_t& depth() noexcept;
		const real_t& depth() const noexcept;

		BBox3 bounds() const noexcept;
		BBox3 skewed_bounds(real_t delta) const noexcept;
		void calculate_inertia();

		vec3_t to_local_vector(const vec3_t& v) const noexcept;
		vec3_t to_world_vector(const vec3_t& v) const noexcept;
		vec3_t to_local(const vec3_t& v) const noexcept;
		vec3_t to_world(const vec3_t& v) const noexcept;

		Transform3 transform() const noexcept;
	};
}