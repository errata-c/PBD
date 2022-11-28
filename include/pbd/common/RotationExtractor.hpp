#pragma once
#include <array>
#include <pbd/common/Types.hpp>

namespace pbd {
	// Class to handle the robust extraction of rotation matricies from deformable objects.
	// This class will calculate a matrix that represents the rotation of a deformed tetrahedron.
	// If the bones for your mesh are not prealigned with the initial rotation of the tetrahedron, you have to apply an inverse transform to them.
	class RotationExtractor {
	public:
		RotationExtractor();
		RotationExtractor(
			const vec3_t& p0,
			const vec3_t& p1,
			const vec3_t& p2,
			const vec3_t& p3
		);

		void reset(
			const vec3_t& p0,
			const vec3_t& p1,
			const vec3_t& p2,
			const vec3_t& p3);

		void update(
			const vec3_t& p0,
			const vec3_t& p1,
			const vec3_t& p2,
			const vec3_t& p3);

		// The extracted rotation.
		glm::mat3 rotation;

		// The previous solution quaternion
		quat_t prior;
	};
}