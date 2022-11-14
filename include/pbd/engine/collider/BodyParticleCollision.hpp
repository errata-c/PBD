#pragma once
#include <pbd/engine/collider/Collision.hpp>

namespace pbd {
	class Particle;
	class RigidBody;

	struct PointNorm {
		glm::vec3 point, normal;
		float distance;
	};
	PointNorm sdCapsule(const glm::vec3& p, float r, float h);
	PointNorm sdBox(const glm::vec3& p, const glm::vec3& b);
	PointNorm sdSphere(const glm::vec3& p, float r);
	PointNorm sdCylinder(const glm::vec3& p, float r, float h);

	std::optional<Collision> capsule_particle_collide(const RigidBody& p0, const Particle& p1);
	std::optional<Collision> cylinder_particle_collide(const RigidBody& p0, const Particle& p1);
	std::optional<Collision> obb_particle_collide(const RigidBody& p0, const Particle& p1);
	std::optional<Collision> sphere_particle_collide(const RigidBody& p0, const Particle& p1);
}