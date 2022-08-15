#pragma once
#include <cinttypes>
#include <array>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;

	struct ConstraintTetraVolume {
		static constexpr Constraint Kind = Constraint::TetraVolume;
		static constexpr std::array<std::array<int32_t, 3>, 4> faceOrder{ {
			{0,1,3},
			{0,2,3},
			{0,3,1},
			{0,1,2}
		}};

		std::array<int32_t, 4> ids;
		float volume;
		float compliance;

		void eval(Engine & engine, float rdt2) const;

		void remap(int32_t offset);
	};

	static_assert(alignof(ConstraintTetraVolume) == alignof(int32_t));
	static_assert(SizeOf(Constraint::TetraVolume) == sizeof(ConstraintTetraVolume));
}