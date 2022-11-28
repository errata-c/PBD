#pragma once
#include <cinttypes>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>
#include <pbd/engine/RigidBody.hpp>
#include <pbd/engine/Particle.hpp>

namespace pbd {
	// Can't make an array of references, so pointers are necessary
	void apply_positional_correction(
		std::array<RigidBody*, 2> b,
		std::array<const vec3_t*, 2> r,
		vec3_t n,
		real_t alpha
	);
	// Positional correction involving a rigid body and a particle.
	void apply_positional_correction(
		RigidBody* b,
		Particle* p,
		const vec3_t* r,
		vec3_t n,
		real_t alpha
	);
	void apply_angular_correction(
		std::array<RigidBody*, 2> b,
		vec3_t n,
		real_t alpha
	);

	// This function is slow, there may be a better way to do this.
	void apply_angular_limit(
		std::array<RigidBody*, 2> b,
		const vec3_t& n,
		const vec3_t& n1,
		const vec3_t& n2,
		real_t min_angle,
		real_t max_angle,
		real_t alpha
	);

	/*
	void apply_angular_limit(
		std::array<RigidBody*, 2> b,
		const vec3_t& n,
		const vec3_t& n1,
		const vec3_t& n2,
		real_t min_angle,
		real_t max_angle,
		real_t alpha
	);
	*/
}