#pragma once
#include <cinttypes>
#include <array>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;
	
	// Very similar to distance constraint, but only active when >= 0
	struct CollideParticle {
		//static constexpr Constraint Kind = Constraint::CollideParticle;

		std::array<int32_t, 2> ids;
		real_t compliance;

		void eval(Engine& engine, real_t rdt2) const;

		void remap(int32_t offset);
	};

	static_assert(alignof(CollideParticle) == alignof(int32_t));
}