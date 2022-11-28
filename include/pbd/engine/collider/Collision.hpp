#pragma once
#include <array>
#include <optional>
#include <pbd/common/Types.hpp>

namespace pbd {
	struct Collision {
		std::array<vec3_t, 2> contacts;
		vec3_t normal;
		real_t depth;
	};
}