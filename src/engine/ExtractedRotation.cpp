#include <pbd/engine/ExtractedRotation.hpp>
#include <pbd/engine/Engine.hpp>

namespace pbd {
	static glm::quat genericExtract(const Engine& engine, const glm::mat3 & A, const glm::quat& _q, int iters) {
		glm::quat q = _q;
		for (int i = 0; i < iters; ++i) {
			glm::mat3 R = glm::mat3_cast(q);
			glm::vec3 omega =
				(glm::cross(R[0], A[0]) +
					glm::cross(R[1], A[1]) +
					glm::cross(R[2], A[2])) *
				(1.f / (std::abs(
					glm::dot(R[0], A[0]) +
					glm::dot(R[0], A[0]) +
					glm::dot(R[0], A[0])
				) + 1e-6f));

			float w = glm::length(omega);
			if (w < 1e-5f) {
				break;
			}

			q = glm::angleAxis(w, omega / w) * q;
			q = glm::normalize(q);
		}

		return q;
	}
	static glm::mat3 tetraMatrix(const Engine& engine, const std::array<int, 4>& ids) {
		glm::vec3 
			a1 = engine.particle.pos[ids[1]] - engine.particle.pos[ids[0]],
			a2 = engine.particle.pos[ids[2]] - engine.particle.pos[ids[0]],
			a3 = engine.particle.pos[ids[3]] - engine.particle.pos[ids[0]];

		return glm::mat3(a1, a2, a3);
	}

	ExtractedRotation::ExtractedRotation()
		: ids{0,0,0}
		, rotation(1.f)
	{}
	ExtractedRotation::ExtractedRotation(int i0, int i1, int i2, int i3, const Engine& engine)
		: ids{i0, i1, i2, i3}
	{
		initialize(engine);
	}

	void ExtractedRotation::reset(int i0, int i1, int i2, int i3, const Engine& engine) {
		ids = {i0, i1, i2, i3};
		initialize(engine);
	}

	void ExtractedRotation::extract(const Engine& engine) {
		// Warm start
		prior = genericExtract(engine, tetraMatrix(engine, ids), prior, 5);
		rotation = glm::mat3_cast(prior);
	}

	void ExtractedRotation::initialize(const Engine& engine) {
		// Calculate the matrix from the tetrahedra
		glm::quat q(1.f, 0.f, 0.f, 0.f);
		prior = genericExtract(engine, tetraMatrix(engine, ids), q, 15);
		rotation = glm::mat3_cast(prior);
	}

	
}