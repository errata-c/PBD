#pragma once
#include <array>
#include <glm/vec3.hpp>

namespace pbd {
	class Engine;

	struct CollideTriangle {
		// Triangle particle ids.
		std::array<int, 3> ids;
		// Colliding particle id.
		int q;
		// Projection distance from plane.
		float h;
	};
}