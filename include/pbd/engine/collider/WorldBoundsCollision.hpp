#pragma once
#include <pbd/engine/collider/Collision.hpp>

namespace pbd {
	class Engine;
	class RigidBody;
	class Particle;

	std::optional<Collision> bounds_particle_collide(const Engine& engine, const Particle& p0);
	std::optional<Collision> bounds_capsule_collide(const Engine& engine, const RigidBody& b0);
	std::optional<Collision> bounds_cylinder_collide(const Engine& engine, const RigidBody& b0);
	std::optional<Collision> bounds_obb_collide(const Engine& engine, const RigidBody& b0);
	std::optional<Collision> bounds_sphere_collide(const Engine& engine, const RigidBody& b0);

	std::optional<Collision> plane_particle_collide(const vec3_t & position, const vec3_t& normal, const Particle& p0);
	std::optional<Collision> plane_capsule_collide(const vec3_t& position, const vec3_t& normal, const RigidBody& b0);
	std::optional<Collision> plane_cylinder_collide(const vec3_t& position, const vec3_t& normal, const RigidBody& b0);
	std::optional<Collision> plane_obb_collide(const vec3_t& position, const vec3_t& normal, const RigidBody& b0);
	std::optional<Collision> plane_sphere_collide(const vec3_t& position, const vec3_t& normal, const RigidBody& b0);
}