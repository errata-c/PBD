#pragma once
#include <pbd/engine/collider/Collision.hpp>

namespace pbd {
	class RigidBody;

	std::optional<Collision> capsule_capsule_collide(const RigidBody& p0, const RigidBody& p1);
	std::optional<Collision> capsule_cylinder_collide(const RigidBody& p0, const RigidBody& p1);
	std::optional<Collision> capsule_obb_collide(const RigidBody& p0, const RigidBody& p1);
	std::optional<Collision> capsule_sphere_collide(const RigidBody& p0, const RigidBody& p1);

	std::optional<Collision> cylinder_cylinder_collide(const RigidBody& p0, const RigidBody& p1);
	std::optional<Collision> cylinder_obb_collide(const RigidBody& p0, const RigidBody& p1);
	std::optional<Collision> cylinder_sphere_collide(const RigidBody& p0, const RigidBody& p1);

	std::optional<Collision> obb_obb_collide(const RigidBody& p0, const RigidBody& p1);
	std::optional<Collision> obb_sphere_collide(const RigidBody& p0, const RigidBody& p1);

	std::optional<Collision> sphere_sphere_collide(const RigidBody& p0, const RigidBody& p1);
}