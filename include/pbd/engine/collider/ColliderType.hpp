#pragma once
#include <cinttypes>
#include <pbd/engine/RigidBody.hpp>

namespace pbd {
	enum class ColliderType : uint32_t {
		ParticleParticle,

		CapsuleParticle,
		CylinderParticle,
		OBBParticle,
		SphereParticle,

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

		WorldBoundsParticle,
		WorldBoundsCapsule,
		WorldBoundsCylinder,
		WorldBoundsOBB,
		WorldBoundsSphere,
	};
	
	ColliderType particle_particle_collider() noexcept;
	ColliderType shape_particle_collider(Shape lh) noexcept;
	ColliderType shape_shape_collider(Shape lh, Shape rh) noexcept;
}