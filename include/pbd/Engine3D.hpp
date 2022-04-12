#pragma once
#include <vector>
#include <glm/vec3.hpp>

namespace pbd {
	class Engine3D {
	public:

		glm::vec3 gravity;
		float dt;
		int numSubsteps;
		float friction;


		// We should allow for setting the position and the velocity in between steps.
		// We should allow the mass to change as well.
		// prevPos is entirely for internal use.


		std::vector<glm::vec3> pos, prevPos, velocity;
		std::vector<float> invMass;

	private:
		
	};
}