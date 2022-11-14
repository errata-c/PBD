#include <pbd/engine/constraint/body/AttachParticle.hpp>
#include <pbd/engine/Engine.hpp>

#include <pbd/engine/constraint/body/Utils.hpp>
#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CAttachParticle::serialize(const CAttachParticle& in, std::string& output) {
		ez::serialize::i32(in.body_id, output);
		ez::serialize::i32(in.particle_id, output);

		ez::serialize::f32(in.r[0], output);
		ez::serialize::f32(in.r[1], output);
		ez::serialize::f32(in.r[2], output);

		ez::serialize::f32(in.compliance, output);
	}
	const char* CAttachParticle::deserialize(const char* first, const char* last, CAttachParticle& out) {
		first = ez::deserialize::i32(first, last, out.body_id);
		first = ez::deserialize::i32(first, last, out.particle_id);

		first = ez::deserialize::f32(first, last, out.r[0]);
		first = ez::deserialize::f32(first, last, out.r[1]);
		first = ez::deserialize::f32(first, last, out.r[2]);

		first = ez::deserialize::f32(first, last, out.compliance);

		return first;
	}

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