#include <pbd/engine/collider/ParticleParticleCollision.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/Particle.hpp>
#include <pbd/common/Utils.hpp>

namespace pbd {
	std::optional<Collision> particle_particle_collide(const Particle & p0, const Particle& p1) {
		real_t threshold = p0.radius + p1.radius;
		threshold = threshold * threshold;

		vec3_t normal = p1.position - p0.position;
		real_t dist = glm::dot(normal, normal);

		if (dist < threshold) {
			dist = std::sqrt(dist);
			if (dist < 1e-5f) {
				return std::nullopt;
			}
			normal /= dist;

			Collision collide;
			collide.normal = normal;
			collide.contacts[0] = normal * p0.radius;
			collide.contacts[1] = -normal * p1.radius;

			return collide;
		}
		else {
			return std::nullopt;
		}
	}
}

/*
std::array<Particle*, 2> p{
			&engine.particles.list[ids[0]],
			&engine.particles.list[ids[1]]
		};

		real_t w0 = p[0]->imass;
		real_t w1 = p[1]->imass;

		real_t w = w0 + w1 + compliance * rdt2;
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// Grads 4x3
		vec3_t grad = p[0]->position - p[1]->position;
		real_t length = glm::length(grad);
		real_t distance = p[0]->radius + p[1]->radius;
		real_t C = length - distance;
		if (C >= 0.f || length < 1e-5f) {
			// Do nothing when the constraint is above zero
			// Also prevent divide by zero
			return;
		}

		grad /= length;

		real_t lambda = C / w;

		// Update the positions for the next constraint to use.
		p[0]->position += -lambda * w0 * grad;
		p[1]->position +=  lambda * w1 * grad;

		// Frictional delta
		// [(x0 - prev x0) - (x1 - prev x1)] perpendicular to normal (x0 - x1, see grad above)

		// Current positional deltas for the substep.
		vec3_t px0 = p[0]->position - engine.particles.prevPos[ids[0]];
		vec3_t px1 = p[1]->position - engine.particles.prevPos[ids[1]];

		// Friction normal is the grad
		px0 = perpendicular(px0, grad);
		px1 = perpendicular(px1, grad);

		px0 = friction_delta(px0, -C, engine.static_friction, engine.kinetic_friction);
		px1 = friction_delta(px1, -C, engine.static_friction, engine.kinetic_friction);

		p[0]->position += w0 * w * px0;
		p[1]->position += w1 * w * px1;
*/