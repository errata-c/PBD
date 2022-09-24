#pragma once
#include <cinttypes>
#include <array>
#include <pbd/common/Types.hpp>

#include <glm/mat3x3.hpp>

namespace pbd {
	class Engine;

	// More costly than the other Tetra constraint, but does not require distance constraints.
	// Technically two constraints in one, we could allow for two compliance values.
	// Hydrostatic constraint is volume, Deviatoric constraint is essentially shear.
	struct ConstraintNHTetraVolume {
		static constexpr Constraint Kind = Constraint::NHTetraVolume;

		ConstraintNHTetraVolume();
		ConstraintNHTetraVolume(const std::array<int32_t, 4> _ids, const glm::mat3& _irest, float hydro, float devia);
		ConstraintNHTetraVolume(
			const std::array<int32_t, 4> _ids, 
			const glm::vec3& p0,
			const glm::vec3& p1,
			const glm::vec3& p2,
			const glm::vec3& p3,
			float hydro, 
			float devia);

		std::array<int32_t, 4> ids;
		glm::mat3 inv_rest;
		float hydrostatic_compliance, deviatoric_compliance;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
	};

	static_assert(alignof(ConstraintNHTetraVolume) == alignof(int32_t));
	static_assert(SizeOf(Constraint::NHTetraVolume) == sizeof(ConstraintNHTetraVolume));
}