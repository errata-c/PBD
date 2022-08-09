#pragma once
#include <glm/vec3.hpp>
#include <array>

namespace pbd {
	class Engine;

	// This constrain is very expensive. Probably not worth it.
	struct Bend {
		std::array<int, 4> ids;
		float angle;
	};
}