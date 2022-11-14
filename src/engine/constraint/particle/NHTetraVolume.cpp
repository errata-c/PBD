#include <pbd/engine/constraint/particle/NHTetra.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>
#include <glm/common.hpp>
#include <glm/gtx/norm.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void CNHTetra::serialize(const CNHTetra& in, std::string& output) {
		ez::serialize::i32(in.ids[0], output);
		ez::serialize::i32(in.ids[1], output);
		ez::serialize::i32(in.ids[2], output);
		ez::serialize::i32(in.ids[3], output);

		for (int c = 0; c < 3; ++c) {
			for (int r = 0; r < 3; ++r) {
				ez::serialize::f32(in.inv_rest[c][r], output);
			}
		}
		
		ez::serialize::f32(in.hydrostatic_compliance, output);
		ez::serialize::f32(in.deviatoric_compliance, output);
	}
	const char* CNHTetra::deserialize(const char* first, const char* last, CNHTetra& out) {
		first = ez::deserialize::i32(first, last, out.ids[0]);
		first = ez::deserialize::i32(first, last, out.ids[1]);
		first = ez::deserialize::i32(first, last, out.ids[2]);
		first = ez::deserialize::i32(first, last, out.ids[3]);

		for (int c = 0; c < 3; ++c) {
			for (int r = 0; r < 3; ++r) {
				first = ez::deserialize::f32(first, last, out.inv_rest[c][r]);
			}
		}

		first = ez::deserialize::f32(first, last, out.hydrostatic_compliance);
		first = ez::deserialize::f32(first, last, out.deviatoric_compliance);
		return first;
	}

	CNHTetra::CNHTetra()
		: ids{0,0,0,0}
		, hydrostatic_compliance(1.f)
		, deviatoric_compliance(1.f)
		, inv_rest(1.f)
	{}
	CNHTetra::CNHTetra(const std::array<int32_t, 4> _ids, const glm::mat3& _irest, float hydro, float devia)
		: ids(_ids)
		, inv_rest(_irest)
		, hydrostatic_compliance(hydro)
		, deviatoric_compliance(devia)
	{}
	CNHTetra::CNHTetra(
		const std::array<int32_t, 4> _ids,
		const glm::vec3& p0,
		const glm::vec3& p1,
		const glm::vec3& p2,
		const glm::vec3& p3,
		float hydro,
		float devia)
		: ids(_ids)
		, hydrostatic_compliance(hydro)
		, deviatoric_compliance(devia)
	{
		inv_rest = glm::mat3(
			p1 - p0,
			p2 - p0,
			p3 - p0
		);
		inv_rest = glm::inverse(inv_rest);
	}

	static void apply(
		std::array<Particle*, 4> & p, 
		std::array<glm::vec3, 4> & grads, 
		float C, 
		float rdt2,
		float compliance) 
	{
		if(std::abs(C) < 1e-4f) {
			return;
		}

		grads[0] = -grads[1] - grads[2] - grads[3];

		float w = 0.f;
		for (int i = 0; i < 4; ++i) {
			w += glm::length2(grads[i]) * p[i]->imass;
		}

		if (std::abs(w) < 1e-4f) {
			return;
		}

		float lambda = -C / (w + compliance * rdt2);
		for (int i = 0; i < 4; ++i) {
			p[i]->position += grads[i] * lambda * p[i]->imass;
		}
	}
	static float matIJ(const glm::mat3& mat, int i, int j) {
		return mat[j][i];
	}

	void CNHTetra::eval(Engine& engine, float rdt2) const {
		std::array<Particle*, 4> p;
		for (int i = 0; i < ids.size(); ++i) {
			p[i] = &engine.particles.list[ids[i]];
		}

		glm::mat3 P = glm::mat3{
			(p[1]->position) - (p[0]->position),
			(p[2]->position) - (p[0]->position),
			(p[3]->position) - (p[0]->position)
		};

		glm::mat3 F = P * inv_rest;

		std::array<glm::vec3, 4> grads;

		for (int i = 0; i < 3; ++i) {
			grads[i + 1] = 
				2.f * matIJ(inv_rest, i, 0) * F[0] +
				2.f * matIJ(inv_rest, i, 1) * F[1] +
				2.f * matIJ(inv_rest, i, 2) * F[2];
		}
		
		float C = glm::length2(F[0]) + glm::length2(F[1]) + glm::length2(F[2]) - 3.f;

		apply(p, grads, C, rdt2, deviatoric_compliance);

		P = glm::mat3{
		   (p[1]->position) - (p[0]->position),
		   (p[2]->position) - (p[0]->position),
		   (p[3]->position) - (p[0]->position)
		};

		F = P * inv_rest;

		grads.fill(glm::vec3(0));

		glm::vec3 dF = glm::cross(F[1], F[2]);
		for (int i = 0; i < 3; ++i) {
			grads[i + 1] += dF * matIJ(inv_rest, i, 0);
		}

		dF = glm::cross(F[2], F[0]);
		for (int i = 0; i < 3; ++i) {
			grads[i + 1] += dF * matIJ(inv_rest, i, 1);
		}

		dF = glm::cross(F[0], F[1]);
		for (int i = 0; i < 3; ++i) {
			grads[i + 1] += dF * matIJ(inv_rest, i, 2);
		}

		C = glm::determinant(F) - 1.f;
		
		apply(p, grads, C, rdt2, hydrostatic_compliance);
	}

	void CNHTetra::remap(int32_t offset) {
		for (int32_t& id : ids) {
			id += offset;
		}
	}
	void CNHTetra::transform(const Transform3& form) {
		glm::mat3 tmp = glm::inverse(inv_rest);
		tmp *= form.size;
		inv_rest = glm::inverse(tmp);
	}
}