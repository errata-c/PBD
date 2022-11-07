#pragma once
#include <cinttypes>
#include <pbd/Types.hpp>

namespace pbd {
	class Engine;

	struct ConstraintDistance {
		static constexpr Constraint Kind = Constraint::DampedDistance;

		void eval(Engine& engine, float rdt2) const;

		int32_t p0, p1;
		float length;
		float compliance;
		float damping;
	};

	static_assert(alignof(ConstraintDistance) == alignof(int32_t));
}