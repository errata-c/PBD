#pragma once
#include <cinttypes>
#include <pbd/Types.hpp>

namespace pbd {
	class Engine;

	struct ConstraintDistance {
		static constexpr Constraint Kind = Constraint::Distance;

		int32_t p0, p1;
		float initialLength;

		void eval(Engine& engine) const;
	};

	static_assert(alignof(ConstraintDistance) == alignof(int32_t));
}