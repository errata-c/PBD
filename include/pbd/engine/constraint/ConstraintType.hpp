#pragma once
#include <cinttypes>
#include <cstddef>

namespace pbd {
	enum class ConstraintType : uint32_t {
		// Constraints with one id:
		CollidePlane,

		// Constraints with two ids:
		Distance,

		Align,
		AttachBody,
		AttachParticle,
		HingeJoint,
		PrismaticJoint,
		SphereJoint,

		// Constraints with four ids:
		Tetra,
		NHTetra,

		// Make sure to update these when needed!
		_Last1ID = CollidePlane,
		_Last2ID = SphereJoint,
		_Last4ID = NHTetra,
	};
	size_t SizeOf(ConstraintType type) noexcept;
	constexpr size_t NumIds(ConstraintType type) noexcept {
		return (static_cast<uint32_t>(type) <= static_cast<uint32_t>(ConstraintType::_Last1ID)) ? 1 :
			((static_cast<uint32_t>(type) <= static_cast<uint32_t>(ConstraintType::_Last2ID)) ? 2 : 4);
	}
}