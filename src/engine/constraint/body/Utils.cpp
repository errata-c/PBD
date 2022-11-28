#include <pbd/engine/constraint/body/Utils.hpp>

namespace pbd {
	// As long as the update timestep is relatively small, the orientations don't have to renormalized here
	// Although, a small iterative renormalization could help.
	static constexpr bool use_iterative_renormalize = true;
	static quat_t iterative_renormalize(const quat_t & rot) noexcept {
		real_t k = glm::dot(rot, rot);
		return rot * (real_t(1) - (k - real_t(1)) / (k + real_t(1)));
	}

	void apply_positional_correction(
		// Rigid bodies to update
		std::array<RigidBody*, 2> b,

		// Local space attachment points
		std::array<const vec3_t*, 2> r,

		// World space correction vector
		vec3_t n,

		// Compliance * dt^2
		real_t alpha
	) {
		/*
		real_t wsum = b[0]->imass + b[1]->imass;
		if (wsum < eps()) {
			// No update needed.
			// This is avoidable, maybe trigger an assertion?
			assert(false);
			return;
		}
		*/

		real_t c = glm::length(n);

		// Only apply the positional correction if magnitude is large enough (avoid divide by zero)
		if (c < eps()) {
			return;
		}

		n /= c;

		// Generalized masses
		std::array<real_t, 2> w;
		// Local space normal, and cross normal
		std::array<vec3_t, 2> crn;

		// Calculate the generalized inverse masses (w)
		for (int i = 0; i < 2; ++i) {
			// Project 'n' into the rest state of the body
			vec3_t pn = b[i]->to_local_vector(n);
			crn[i] = glm::cross(*r[i], pn);

			w[i] = b[i]->imass + glm::dot(crn[i] * b[i]->inverse_inertia, crn[i]);
		}

		// Calculate the lagrange multiplier delta
		std::array<real_t, 2> dlambda;
		dlambda[0] = c / (w[0] + w[1] + alpha);
		dlambda[1] = -dlambda[0];

		// Update the state of the bodies:
		for (int i = 0; i < 2; ++i) {
			// Use the world space 'n' value!
			// While dlambda was calculated using local space values, it is still valid for world space.
			b[i]->position += dlambda[i] * n * b[i]->imass;

			// lambda is magnitude, crn is component of normal perpendicular to attach point.
			vec3_t tmp = real_t(0.5) * dlambda[i] * crn[i] * b[i]->inverse_inertia;

			// This is a local space rotation, no need to convert anything.
			b[i]->orientation += b[i]->orientation * quat_t(0.f, tmp.x, tmp.y, tmp.z);
		}

		if constexpr(use_iterative_renormalize) {
			for (int i = 0; i < 2; ++i) {
				b[i]->orientation = iterative_renormalize(b[i]->orientation);
			}
		}
	}

	void apply_positional_correction(
		RigidBody* b,
		Particle* p,
		const vec3_t* r,
		vec3_t n,
		real_t alpha
	) {
		real_t c = glm::length(n);

		// Only apply the positional correction if magnitude is large enough (avoid divide by zero)
		if (c < 1e-5f) {
			return;
		}

		n /= c;

		std::array<real_t, 2> w;
		vec3_t pn, crn;

		// Calculate the generalized inverse masses (w)
		{
			// Project 'n' into the rest state of the body
			pn = b->to_local_vector(n);
			crn = glm::cross(*r, pn);

			w[0] = b->imass + glm::dot(crn * b->inverse_inertia, crn);
		}
		w[1] = p->imass;

		// Calculate the lagrange multiplier delta
		real_t dlambda = -c / (w[0] + w[1] + alpha);

		// Update the state of the body:
		{
			b->position += dlambda * pn * b->imass;

			// Conservation angular momentum?
			vec3_t tmp = real_t(0.5) * dlambda * crn * b->inverse_inertia;

			b->orientation += b->orientation * quat_t(0.f, tmp.x, tmp.y, tmp.z);
		}
		p->position -= dlambda * n * w[1];

		if constexpr (use_iterative_renormalize) {
			b->orientation = iterative_renormalize(b->orientation);
		}
	}

	void apply_angular_correction(
		std::array<RigidBody*, 2> b,
		vec3_t n,
		real_t alpha
	) {
		real_t c = glm::length(n);

		// Only apply the positional correction if magnitude is large enough (avoid divide by zero)
		if (c < 1e-5) {
			return;
		}

		n /= c;

		std::array<real_t, 2> w;
		std::array<vec3_t, 2> pn;

		// Calculate the generalized inverse masses (w)
		for (int i = 0; i < 2; ++i) {
			// Project 'n' into the rest state of the body
			pn[i] = b[i]->to_local_vector(n);

			w[i] = b[i]->imass + glm::dot(pn[i] * b[i]->inverse_inertia, pn[i]);
		}

		// Calculate the lagrange multiplier delta
		std::array<real_t, 2> dlambda;
		dlambda[0] = -c / (w[0] + w[1] + alpha);
		dlambda[1] = -dlambda[0];

		// Update the state of the bodies:
		for (int i = 0; i < 2; ++i) {
			vec3_t p = dlambda[i] * pn[i];

			// Conservation angular momentum?
			vec3_t tmp = real_t(0.5) * p * b[i]->inverse_inertia;

			b[i]->orientation += b[i]->orientation * quat_t(0.f, tmp.x, tmp.y, tmp.z);
		}

		if constexpr(use_iterative_renormalize) {
			for (int i = 0; i < 2; ++i) {
				b[i]->orientation = iterative_renormalize(b[i]->orientation);
			}
		}
	}

	void apply_angular_limit(
		std::array<RigidBody*, 2> b,
		const vec3_t& n, // Common rotation axis
		const vec3_t& n1, // Axis of body 0
		const vec3_t& n2, // Axis of body 1
		real_t min_angle,
		real_t max_angle,
		real_t alpha
	) {
		static constexpr real_t pi = 3.14159265358979323846;
		static constexpr real_t tau = pi * 2.f;

		real_t angle = std::asin(glm::dot(glm::cross(n1, n2), n));

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
			vec3_t tmp = glm::rotate(glm::angleAxis(angle, n), n1);
			apply_angular_correction(b, glm::cross(tmp, n2), alpha);
		}
	}
}