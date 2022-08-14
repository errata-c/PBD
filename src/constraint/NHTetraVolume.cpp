#pragma once
#include <pbd/constraint/NHTetraVolume.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/geometric.hpp>
#include <glm/common.hpp>
#include <glm/gtx/norm.hpp>

namespace pbd {
	static void apply(const ConstraintNHTetraVolume & tet, Engine & engine, std::array<glm::vec3*, 4> & x, std::array<glm::vec3, 4> & grads, float C, float rdt2) {
		if(std::abs(C) < 1e-4f) {
			return;
		}

		grads[0] = -grads[1] - grads[2] - grads[3];

		float w = 0.f;
		for (int i = 0; i < 4; ++i) {
			w += glm::length2(grads[i]) * engine.particle.invMass[tet.ids[i]];
		}

		if (std::abs(w) < 1e-4f) {
			return;
		}

		float lambda = -C / (w + tet.compliance * rdt2);
		for (int i = 0; i < 4; ++i) {
			*x[i] += grads[i] * lambda * engine.particle.invMass[tet.ids[i]];
		}
	}
	static float matIJ(const glm::mat3& mat, int i, int j) {
		return mat[j][i];
	}

	void ConstraintNHTetraVolume::eval(Engine& engine, float rdt2) const {
		std::array<glm::vec3*, 4> x;
		// Fetch the addresses of the positions
		for (int i = 0; i < ids.size(); ++i) {
			x[i] = &engine.particle.pos[ids[i]];
		}

		glm::mat3 P{
			(*x[1]) - (*x[0]),
			(*x[2]) - (*x[0]),
			(*x[3]) - (*x[0])
		};

		glm::mat3 F = P * invRestPose;

		std::array<glm::vec3, 4> grads;

		for (int i = 0; i < 3; ++i) {
			grads[i + 1] = 
				2.f * matIJ(invRestPose, i, 0) * F[0] +
				2.f * matIJ(invRestPose, i, 1) * F[1] +
				2.f * matIJ(invRestPose, i, 2) * F[2];
		}
		
		float C = glm::length2(F[0]) + glm::length2(F[1]) + glm::length2(F[2]) - 3.f;

		apply(*this, engine, x, grads, C, rdt2);


		P[0] = (*x[1]) - (*x[0]);
		P[1] = (*x[2]) - (*x[0]);
		P[2] = (*x[3]) - (*x[0]);

		F = P * invRestPose;

		grads.fill(glm::vec3(0));

		glm::vec3 dF = glm::cross(F[1], F[2]);
		for (int i = 0; i < 3; ++i) {
			grads[i + 1] += dF * matIJ(invRestPose, i, 0);
		}

		dF = glm::cross(F[2], F[0]);
		for (int i = 0; i < 3; ++i) {
			grads[i + 1] += dF * matIJ(invRestPose, i, 1);
		}

		dF = glm::cross(F[0], F[1]);
		for (int i = 0; i < 3; ++i) {
			grads[i + 1] += dF * matIJ(invRestPose, i, 2);
		}

		C = glm::determinant(F) - 1.f;
		
		apply(*this, engine, x, grads, C, rdt2);
	}

	void ConstraintNHTetraVolume::remap(int32_t offset) {
		for (int32_t& id : ids) {
			id += offset;
		}
	}
}