#include <pbd/engine/constraint/body/AttachParticle.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>
#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CAttachParticle::eval(Engine& engine, float rdt2) const {
		float alpha = (compliance * rdt2);

		RigidBody* body = &engine.bodies.list[body_id];
		Particle* particle = &engine.particles.list[particle_id];

		apply_positional_correction(
		body,
		particle,
		&r,
		particle->position - body->to_world(r),
		alpha);
	}

	void CAttachParticle::remap(int32_t offset) {
		// Oops. What do we do about changing particle and body indices?
	}
	void CAttachParticle::transform(const Transform3& form) {

	}
}