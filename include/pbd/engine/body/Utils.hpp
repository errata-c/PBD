#pragma once
#include <cinttypes>
#include <array>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>
#include <pbd/engine/body/RigidBody.hpp>

namespace pbd {
	// Can't make an array of references, so pointers are necessary
	void apply_positional_correction(
		std::array<RigidBody*, 2> b,
		std::array<const glm::vec3*, 2> r,
		glm::vec3 n,
		float alpha
	);
	void apply_angular_correction(
		std::array<RigidBody*, 2> b,
		glm::vec3 n,
		float alpha
	);

	// This function is slow, there may be a better way to do this.
	void apply_angular_limit(
		std::array<RigidBody*, 2> b,
		const glm::vec3& n,
		const glm::vec3& n1,
		const glm::vec3& n2,
		float min_angle,
		float max_angle,
		float alpha
	);

	/*
	void apply_angular_limit(
		std::array<RigidBody*, 2> b,
		const glm::vec3& n,
		const glm::vec3& n1,
		const glm::vec3& n2,
		float min_angle,
		float max_angle,
		float alpha
	);
	*/
}