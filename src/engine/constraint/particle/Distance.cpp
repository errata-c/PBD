#include <pbd/engine/constraint/particle/Distance.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CDistance::serialize(const CDistance& in, std::string& output) {
		ez::serialize::i32(in.p0, output);
		ez::serialize::i32(in.p1, output);
		ez::serialize::f32(in.length, output);
		ez::serialize::f32(in.compliance, output);
	}
	const char* CDistance::deserialize(const char* first, const char* last, CDistance& out) {
		first = ez::deserialize::i32(first, last, out.p0);
		first = ez::deserialize::i32(first, last, out.p1);
		first = ez::deserialize::f32(first, last, out.length);
		return ez::deserialize::f32(first, last, out.compliance);
	}

	void CDistance::eval(Engine& engine, real_t rdt2) const {
		Particle& part0 = engine.particles.list[p0];
		Particle& part1 = engine.particles.list[p1];

		real_t w0 = part0.imass;
		real_t w1 = part1.imass;

		real_t w = w0 + w1 + (compliance * rdt2);
		if (w <= 1e-5f) {
			// Zero mass particles do not move.
			return;
		}

		// references, we are going to modify these in place.
		vec3_t& x0 = part0.position;
		vec3_t& x1 = part1.position;
		
		vec3_t grad = x0 - x1;
		real_t gradLen = glm::length(grad);
		if (gradLen <= 1e-5f) {
			// Zero length gradient means zero length delta. No further work needed.
			return;
		}
		
		grad /= gradLen;

		// The lambda value determines how the movement is to be weighted.
		// -C / (w1*|grad(C0)| + w2*|grad(C1)| + compliance / dt^2)
		real_t C = (gradLen - length);
		real_t lambda = C / w;

		// Update the positions for the next constraint to use.
		x0 -= lambda * w0 * grad;
		x1 += lambda * w1 * grad;
	}

	void CDistance::remap(int32_t particle_offset, int32_t body_offset) {
		p0 += particle_offset;
		p1 += particle_offset;
	}
	void CDistance::transform(const Transform3& form) {
		length *= form.size;
	}
}