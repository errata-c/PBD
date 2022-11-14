#pragma once
#include <pbd/engine/collider/Collision.hpp>


namespace pbd {
	class Particle;
	std::optional<Collision> particle_particle_collide(const Particle& p0, const Particle& p1);
}