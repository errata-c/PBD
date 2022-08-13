#pragma once
#include <cinttypes>
#include <pbd/Types.hpp>

namespace pbd {
	class Engine;
	
	// Very similar to distance constraint, but only active when >= 0
	struct CollideParticle {
		static constexpr Constraint Kind = Constraint::CollideParticle;

		int32_t p0, p1;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
	};

	static_assert(alignof(CollideParticle) == alignof(int32_t));
}