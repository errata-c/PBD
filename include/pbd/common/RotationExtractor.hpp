#pragma once
#include <array>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace pbd {
	// Class to handle the robust extraction of rotation matricies from deformable objects.
	// This class will calculate a matrix that represents the rotation of a deformed tetrahedron.
	// If the bones for your mesh are not prealigned with the initial rotation of the tetrahedron, you have to apply an inverse transform to them.
	class RotationExtractor {
	public:
		RotationExtractor();
		RotationExtractor(
			const glm::vec3& p0,
			const glm::vec3& p1,
			const glm::vec3& p2,
			const glm::vec3& p3
		);

		void reset(
			const glm::vec3& p0,
			const glm::vec3& p1,
			const glm::vec3& p2,
			const glm::vec3& p3);

		void update(
			const glm::vec3& p0,
			const glm::vec3& p1,
			const glm::vec3& p2,
			const glm::vec3& p3);

		// The extracted rotation.
		glm::mat3 rotation;

		// The previous solution quaternion
		glm::quat prior;
	};
}