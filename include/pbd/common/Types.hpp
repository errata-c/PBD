#pragma once
#include <cinttypes>
#include <cstddef>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <glm/gtc/quaternion.hpp>
#include <pbd/common/BBox.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	// Strongly typed ID value for the managed objects
	enum class ObjectID : uint64_t {
		Invalid = uint64_t(-1)
	};

	using real_t = float;
	using quat_t = glm::tquat<real_t>;
	using vec3_t = glm::tvec3<real_t>;
	using vec2_t = glm::tvec2<real_t>;

	using BBox2 = BBox<real_t, 2>;
	using BBox3 = BBox<real_t, 3>;

	using Transform2 = Transform<real_t, 2>;
	using Transform3 = Transform<real_t, 3>;

	constexpr real_t eps() noexcept {
		if constexpr (sizeof(real_t) == 4) {
			return real_t(1e-5);
		}
		else {
			return real_t(1e-13);
		}
	}
}

// Hashing support for phmap
constexpr size_t hash_value(const pbd::ObjectID& id) noexcept {
	return static_cast<size_t>(id);
}