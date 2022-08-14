#pragma once
#include <array>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>
#include <glm/gtc/quaternion.hpp>

namespace pbd {
	class Engine;

	// Class to handle the robust extraction of rotation matricies from deformable objects.
	// This class will calculate a matrix that represents the rotation of a deformed tetrahedron.
	// If the bones for your mesh are not prealigned with the initial rotation of the tetrahedron, you have to apply an inverse transform to them.
	class ExtractedRotation {
	public:
		ExtractedRotation();
		ExtractedRotation(int i0, int i1, int i2, int i3, const Engine & engine);

		void reset(int i0, int i1, int i2, int i3, const Engine & engine);
		
		void extract(const Engine & engine);
		
		// Ids of the tetrahedra this rotation is extracted from.
		std::array<int, 4> ids;
		
		// The extracted rotation.
		glm::mat3 rotation;

		// The previous solution quaternion
		glm::quat prior;
	private:
		void initialize(const Engine & engine);
	};
}