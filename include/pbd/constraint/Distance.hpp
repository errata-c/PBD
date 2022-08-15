#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;

	struct ConstraintDistance {
		static constexpr Constraint Kind = Constraint::Distance;

		void eval(Engine& engine, float rdt2) const;
		
		void remap(int32_t offset);

		int32_t p0, p1;
		float length;
		float compliance;
	};

	static_assert(alignof(ConstraintDistance) == alignof(int32_t));
	static_assert(SizeOf(Constraint::Distance) == sizeof(ConstraintDistance));
}