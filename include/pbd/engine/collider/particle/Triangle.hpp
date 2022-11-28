#pragma once
#include <cinttypes>
#include <array>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;

	struct CollideTriangle {
		// Triangle particle ids.
		std::array<int32_t, 3> ids;
		// Colliding particle id.
		int32_t q;
		// Projection distance from plane.
		real_t h;
	};
}