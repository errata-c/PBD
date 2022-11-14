#pragma once
#include <cinttypes>
#include <cstddef>
#include <pbd/common/BBox.hpp>

namespace pbd {
	enum class ObjectID : uint64_t {
		Invalid = uint64_t(-1)
	};

	using BBox2 = BBox<2, float>;
	using BBox3 = BBox<3, float>;
}

// Hashing support for phmap
constexpr size_t hash_value(const pbd::ObjectID& id) noexcept {
	return static_cast<size_t>(id);
}