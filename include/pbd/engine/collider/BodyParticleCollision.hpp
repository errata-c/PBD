#pragma once
#include <pbd/engine/collider/Collision.hpp>

namespace pbd {
	class Particle;
	class RigidBody;

	struct PointNorm {
		vec3_t point, normal;
		real_t distance;
	};
	PointNorm sdCapsule(const vec3_t& p, real_t r, real_t h);
	PointNorm sdBox(const vec3_t& p, const vec3_t& b);
	PointNorm sdSphere(const vec3_t& p, real_t r);
	PointNorm sdCylinder(const vec3_t& p, real_t r, real_t h);

	std::optional<Collision> capsule_particle_collide(const RigidBody& p0, const Particle& p1);
	std::optional<Collision> cylinder_particle_collide(const RigidBody& p0, const Particle& p1);
	std::optional<Collision> obb_particle_collide(const RigidBody& p0, const Particle& p1);
	std::optional<Collision> sphere_particle_collide(const RigidBody& p0, const Particle& p1);
}