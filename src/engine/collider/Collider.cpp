#include <pbd/engine/collider/Collider.hpp>
#include <pbd/engine/collider/AllColliders.hpp>
#include <pbd/engine/Engine.hpp>

#include <cassert>

namespace pbd {
	Collider::Collider(int32_t i0, int32_t i1, ColliderType _type)
		: type(_type)
		, ids{i0, i1}
	{}

	void Collider::position_solve(Engine& engine, float dt) {
		// This is the constraint formulation of the collider, to be run during the positional constraint phase.


	}
	void Collider::velocity_solve(Engine& engine, float dt) {
		// This is secondary velocity pass, to be run after solving the initial constraints.

		// Its not entirely clear whether or not we need to recompute contacts, or if we are supposed to reuse the previous ones.
		// For now, I'm assuming that we will recompute contacts.
	}

	std::optional<Collision> Collider::collide(Engine& engine) const {
		switch (type) {
		case ColliderType::ParticleParticle:
			return particle_particle_collide(engine.particles.list[ids[0]], engine.particles.list[ids[1]]);

		case ColliderType::CapsuleParticle:
			return capsule_particle_collide(engine.bodies.list[ids[0]], engine.particles.list[ids[1]]);
		case ColliderType::CylinderParticle:
			return cylinder_particle_collide(engine.bodies.list[ids[0]], engine.particles.list[ids[1]]);
		case ColliderType::OBBParticle:
			return obb_particle_collide(engine.bodies.list[ids[0]], engine.particles.list[ids[1]]);
		case ColliderType::SphereParticle:
			return sphere_particle_collide(engine.bodies.list[ids[0]], engine.particles.list[ids[1]]);

		case ColliderType::CapsuleCapsule:
			return capsule_capsule_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::CapsuleCylinder:
			return capsule_cylinder_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::CapsuleOBB:
			return capsule_obb_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::CapsuleSphere:
			return capsule_sphere_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::CylinderCylinder:
			return cylinder_cylinder_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::CylinderOBB:
			return cylinder_obb_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::CylinderSphere:
			return cylinder_sphere_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::OBBOBB:
			return obb_obb_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::OBBSphere:
			return obb_sphere_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);
		case ColliderType::SphereSphere:
			return sphere_sphere_collide(engine.bodies.list[ids[0]], engine.bodies.list[ids[1]]);

		//case ColliderType::WorldBoundsParticle:
		//case ColliderType::WorldBoundsCapsule:
		//case ColliderType::WorldBoundsCylinder:
		//case ColliderType::WorldBoundsOBB:
		//case ColliderType::WorldBoundsSphere:
		
		default:
			assert(false);
			return std::nullopt;
		}
	}
}