#pragma once
#include <cinttypes>
#include <array>
#include <pbd/Types.hpp>

#include <glm/mat3x3.hpp>

namespace pbd {
	class Engine;

	// More costly than the other Tetra constraint, but does not require distance constraints.
	struct ConstraintNHTetraVolume {
		static constexpr Constraint Kind = Constraint::NHTetraVolume;

		std::array<int32_t, 4> ids;
		glm::mat3 invRestPose;
		float compliance;

		void eval(Engine& engine, float rdt2) const;
	};

	static_assert(alignof(ConstraintNHTetraVolume) == alignof(int32_t));
}