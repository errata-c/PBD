#include <pbd/engine/constraint/body/Utils.hpp>

namespace pbd {
	void apply_positional_correction(
		std::array<RigidBody*, 2> b,
		std::array<const glm::vec3*, 2> r,
		glm::vec3 n,
		float alpha
	) {
		float c = glm::length(n);

		// Only apply the positional correction if magnitude is large enough (avoid divide by zero)
		if (c < 1e-5f) {
			return;
		}

		n /= c;

		std::array<float, 2> w;
		std::array<glm::vec3, 2> pn, crn;

		// Calculate the generalized inverse masses (w)
		for (int i = 0; i < 2; ++i) {
			// Project 'n' into the rest state of the body
			pn[i] = b[i]->to_local_vector(n);
			crn[i] = glm::cross(*r[i], pn[i]);

			w[i] = b[i]->imass + glm::dot(crn[i] * b[i]->inertia, crn[i]);
		}

		// Calculate the lagrange multiplier delta
		float dlambda = -c / (w[0] + w[1] + alpha);

		//Negate the second one
		pn[1] = -pn[1];
		crn[1] = -crn[1];

		// Update the state of the bodies:
		for (int i = 0; i < 2; ++i) {
			glm::vec3 p = dlambda * pn[i];

			b[i]->position += p * b[i]->imass;

			// Conservation angular momentum?
			glm::vec3 tmp = dlambda * crn[i] * b[i]->inertia;

			b[i]->orientation += 0.5f * glm::quat(0.f, tmp.x, tmp.y, tmp.z) * b[i]->orientation;
		}

		// Should we renormalized the orientations here?
	}

	void apply_positional_correction(
		RigidBody* b,
		Particle* p,
		const glm::vec3* r,
		glm::vec3 n,
		float alpha
	) {
		float c = glm::length(n);

		// Only apply the positional correction if magnitude is large enough (avoid divide by zero)
		if (c < 1e-5f) {
			return;
		}

		n /= c;

		std::array<float, 2> w;
		glm::vec3 pn, crn;

		// Calculate the generalized inverse masses (w)
		{
			// Project 'n' into the rest state of the body
			pn = b->to_local_vector(n);
			crn = glm::cross(*r, pn);

			w[0] = b->imass + glm::dot(crn * b->inertia, crn);
		}
		w[1] = p->imass;

		// Calculate the lagrange multiplier delta
		float dlambda = -c / (w[0] + w[1] + alpha);

		// Update the state of the body:
		{
			b->position += dlambda * pn * b->imass;

			// Conservation angular momentum?
			glm::vec3 tmp = dlambda * crn * b->inertia;

			b->orientation += 0.5f * glm::quat(0.f, tmp.x, tmp.y, tmp.z) * b->orientation;
		}
		p->position -= dlambda * n * w[1];
	}

	void apply_angular_correction(
		std::array<RigidBody*, 2> b,
		glm::vec3 n,
		float alpha
	) {
		float c = glm::length(n);

		// Only apply the positional correction if magnitude is large enough (avoid divide by zero)
		if (c < 1e-5f) {
			return;
		}

		n /= c;

		std::array<float, 2> w;
		std::array<glm::vec3, 2> pn;

		// Calculate the generalized inverse masses (w)
		for (int i = 0; i < 2; ++i) {
			// Project 'n' into the rest state of the body
			pn[i] = b[i]->to_local_vector(n);

			w[i] = b[i]->imass + glm::dot(pn[i] * b[i]->inertia, pn[i]);
		}

		// Calculate the lagrange multiplier delta
		float dlambda = -c / (w[0] + w[1] + alpha);

		// Negate the second
		pn[1] = -pn[1];

		// Update the state of the bodies:
		for (int i = 0; i < 2; ++i) {
			glm::vec3 p = dlambda * pn[i];

			// Conservation angular momentum?
			glm::vec3 tmp = p * b[i]->inertia;

			b[i]->orientation += 0.5f * glm::quat(0.f, tmp.x, tmp.y, tmp.z) * b[i]->orientation;
		}
	}

	void apply_angular_limit(
		std::array<RigidBody*, 2> b,
		const glm::vec3& n, // Common rotation axis
		const glm::vec3& n1, // Axis of body 0
		const glm::vec3& n2, // Axis of body 1
		float min_angle,
		float max_angle,
		float alpha
	) {
		static constexpr float pi = 3.14159265358979323846;
		static constexpr float tau = pi * 2.f;

		float angle = std::asin(glm::dot(glm::cross(n1, n2), n));

		if (glm::dot(n1, n2) < 0.f) {
			angle = pi - angle;
		}
		if (angle > pi) {
			angle = angle - tau;
		}
		if (angle < -pi) {
			angle = angle + tau;
		}
		if (angle < min_angle || angle > max_angle) {
			angle = std::max(min_angle, std::min(angle, max_angle));
			glm::vec3 tmp = glm::rotate(glm::angleAxis(angle, n), n1);
			apply_angular_correction(b, glm::cross(tmp, n2), alpha);
		}
	}
}