#include <pbd/engine/constraint/particle/Tetra.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CTetra::serialize(const CTetra& in, std::string& output) {
		ez::serialize::i32(in.ids[0], output);
		ez::serialize::i32(in.ids[1], output);
		ez::serialize::i32(in.ids[2], output);
		ez::serialize::i32(in.ids[3], output);

		ez::serialize::f32(in.volume, output);
		ez::serialize::f32(in.compliance, output);
	}
	const char* CTetra::deserialize(const char* first, const char* last, CTetra& out) {
		first = ez::deserialize::i32(first, last, out.ids[0]);
		first = ez::deserialize::i32(first, last, out.ids[1]);
		first = ez::deserialize::i32(first, last, out.ids[2]);
		first = ez::deserialize::i32(first, last, out.ids[3]);

		first = ez::deserialize::f32(first, last, out.volume);
		first = ez::deserialize::f32(first, last, out.compliance);
		return first;
	}

	void CTetra::eval(Engine & engine, real_t rdt2) const {
		std::array<Particle*, 4> p;
		// Fetch the addresses of the positions
		for (int i = 0; i < ids.size(); ++i) {
			p[i] = &engine.particles.list[ids[i]];
		}

		std::array<vec3_t, 4> grads;

		real_t w = 0;
		for (int i = 0; i < 4; ++i) {
			vec3_t t0 = p[faceOrder[i][1]]->position - p[faceOrder[i][0]]->position;
			vec3_t t1 = p[faceOrder[i][2]]->position - p[faceOrder[i][0]]->position;

			grads[i] = glm::cross(t0, t1);
			grads[i] *= (1.0 / 6.0);

			w += p[i]->imass * glm::dot(grads[i], grads[i]);
		}

		w += compliance * rdt2;
		
		if (w <= 1e-5f) {
			// Avoid division by zero
			return;
		}

		// Calculate the volume
		real_t currentVolume = 0;
		{
			vec3_t t0 = p[1]->position - p[0]->position;
			vec3_t t1 = p[2]->position - p[0]->position;
			vec3_t t2 = p[3]->position - p[0]->position;

			vec3_t t3 = glm::cross(t0, t1);
			currentVolume = glm::dot(t3, t2) / 6.0;
		}

		// Apply the deltas
		real_t lambda = -(currentVolume - volume) / w;
		for (int i = 0; i < 4; ++i) {
			p[i]->position += lambda * p[i]->imass * grads[i];
		}
	}

	void CTetra::remap(int32_t particle_offset, int32_t body_offset) {
		for (int32_t& id : ids) {
			id += particle_offset;
		}
	}
	void CTetra::transform(const Transform3& form) {
		volume *= form.size;
	}
}