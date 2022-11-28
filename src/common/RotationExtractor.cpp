#include <pbd/common/RotationExtractor.hpp>
#include <pbd/engine/Engine.hpp>

#include <glm/gtx/orthonormalize.hpp>

namespace pbd {
	template<typename T>
	static glm::tquat<T> genericExtract(const glm::tmat3x3<T>& A, const glm::tquat<T>& _q, int iters) {
		using vec_t = glm::tvec3<T>;
		using quat_t = glm::tquat<T>;
		using mat3_t = glm::tmat3x3<T>;

		quat_t q = _q;
		for (int i = 0; i < iters; ++i) {
			mat3_t R = glm::mat3_cast(q);
			vec_t omega =
				(glm::cross(R[0], A[0]) +
					glm::cross(R[1], A[1]) +
					glm::cross(R[2], A[2])) *
				(1.f / (std::abs(
					glm::dot(R[0], A[0]) +
					glm::dot(R[0], A[0]) +
					glm::dot(R[0], A[0])
				) + 1e-6f));

			T w = glm::length(omega);
			if (w < 1e-5f) {
				break;
			}

			q = glm::angleAxis(w, omega / w) * q;
			q = glm::normalize(q);
		}

		return q;
	}
	static glm::mat3 tetraMatrix(const vec3_t & p0, const vec3_t& p1, const vec3_t& p2, const vec3_t& p3) {
		return glm::mat3(
			p1 - p0,
			p2 - p0,
			p3 - p0
		);
	}

	RotationExtractor::RotationExtractor()
		: rotation(1.f)
		, prior(1.f, 0.f, 0.f, 0.f)
	{}
	RotationExtractor::RotationExtractor(const vec3_t& p0, const vec3_t& p1, const vec3_t& p2, const vec3_t& p3)
	{
		reset(p0, p1, p2, p3);
	}

	void RotationExtractor::reset(const vec3_t& p0, const vec3_t& p1, const vec3_t& p2, const vec3_t& p3) {
		// Calculate the matrix from the tetrahedra
		rotation = tetraMatrix(p0, p1, p2, p3);
		rotation = glm::orthonormalize(rotation);
		prior = glm::quat_cast(rotation);
	}

	void RotationExtractor::update(const vec3_t& p0, const vec3_t& p1, const vec3_t& p2, const vec3_t& p3) {
		// Warm start
		prior = genericExtract(tetraMatrix(p0, p1, p2, p3), prior, 5);
		rotation = glm::mat3_cast(prior);
	}
}