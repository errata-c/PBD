#pragma once
#include <cinttypes>
#include <pbd/common/BBox.hpp>

namespace pbd {
	enum class CollideType : uint32_t {
		ParticleParticle,

		CapsuleCapsule,
		CapsuleCylinder,
		CapsuleOBB,
		CapsuleSphere,
		CylinderCylinder,
		CylinderOBB,
		CylinderSphere,
		OBBOBB,
		OBBSphere,
		SphereSphere,

		CapsuleParticle,
		CylinderParticle,
		OBBParticle,
		SphereParticle,
	};

	enum class Constraint: uint32_t {
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

	size_t SizeOf(Constraint type) noexcept;
	constexpr size_t NumIds(Constraint type) noexcept {
		return (static_cast<uint32_t>(type) <= static_cast<uint32_t>(Constraint::_Last1ID)) ? 1: 
			((static_cast<uint32_t>(type) <= static_cast<uint32_t>(Constraint::_Last2ID)) ? 2: 4);
	}

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