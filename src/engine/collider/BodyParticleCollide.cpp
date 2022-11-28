#include <pbd/engine/collider/BodyParticleCollision.hpp>

#include <pbd/engine/Particle.hpp>
#include <pbd/engine/RigidBody.hpp>
#include <pbd/common/Utils.hpp>

namespace pbd {
	PointNorm sdCapsule(const vec3_t & p, real_t r, real_t h) {
		vec2_t p2(p.x, p.y);
		PointNorm result;

		if (std::abs(p.z) < h * 0.5f) {
			// Below the top of the cylinder segment
			result.normal = p;
			result.normal.z = 0.f;
			result.distance = glm::length(result.normal);
			result.normal /= result.distance;
			result.distance -= r;
			result.point = p - result.normal * result.distance;
		}
		else {
			result.point = vec3_t(0.f, 0.f, glm::sign(p.z) * h * 0.5f);
			result.normal = p - result.point;
			result.distance = glm::length(result.normal);
			result.normal /= result.distance;

			result.point += result.normal * r;
			result.distance -= r;
		}

		return result;
	}
	PointNorm sdBox(const vec3_t & p, const vec3_t & b) {
		vec3_t w = glm::abs(p) - b;
		vec3_t s = glm::sign(p);

		// Index of max value
		int mi = 0;
		// Max value
		real_t g = w.x;
		for (int i = 1; i < 3; ++i) {
			if (w[i] > g) {
				g = w[i];
				mi = i;
			}
		}

		vec3_t q = glm::max(w, 0.f);
		real_t l = glm::length(q);

		PointNorm result;
		if (g >= 0.f) {
			result.distance = l;
			result.normal = s * (q / l);
		}
		else {
			result.distance = g;
			result.normal = vec3_t(0.f);
			result.normal[mi] = s[mi];
		}
		result.point = p - result.normal * result.distance;
		return result;
	}
	PointNorm sdSphere(const vec3_t & p, real_t r) {
		real_t l = glm::length(p);
		PointNorm result;
		result.normal = p / l;
		result.point = result.normal * r;
		result.distance = l - r;
		return result;
	}
	PointNorm sdCylinder(const vec3_t & p, real_t r, real_t h) {
		vec2_t p2(p.x, p.y);
		PointNorm result;

		if (std::abs(p.z) < h * 0.5f) {
			// Below the top of the cylinder
			result.normal = p;
			result.normal.z = 0.f;
			result.distance = glm::length(result.normal);
			result.normal /= result.distance;
			result.distance -= r;
			result.point = p - result.normal * result.distance;
		}
		else {
			real_t d2 = glm::dot(p2, p2);
			if (d2 < r*r) {
				// Above
				result.normal = vec3_t(0.f, 0.f, glm::sign(p.z));
				result.distance = std::abs(p.z) - h * 0.5f;
				result.point = p - result.normal * result.distance;
			}
			else {
				// ringed region
				real_t d = std::sqrt(d2);
				p2 /= d;
				p2 *= r;

				result.point = vec3_t(p2.x, p2.y, glm::sign(p.z) * h * 0.5f);
				result.normal = p - result.point;
				result.distance = glm::length(result.normal);
				result.normal /= result.distance;
			}
		}

		return result;
	}

	std::optional<Collision> capsule_particle_collide(const RigidBody& p0, const Particle& p1) {
		vec3_t tp = p0.to_local(p1.position);
		
		PointNorm pn = sdCapsule(tp, p0.dims[0], p0.dims[1]);

		if (pn.distance < p1.radius) {
			Collision result;
			result.normal = p0.to_world_vector(pn.normal);
			result.contacts[0] = pn.point;
			result.contacts[1] = p1.position - p1.radius * result.normal;
			return result;
		}
		else {
			return std::nullopt;
		}
	}
	std::optional<Collision> cylinder_particle_collide(const RigidBody& p0, const Particle& p1) {
		vec3_t tp = p0.to_local(p1.position);

		PointNorm pn = sdCylinder(tp, p0.dims[0], p0.dims[1]);

		if (pn.distance < p1.radius) {
			Collision result;
			result.normal = p0.to_world_vector(pn.normal);
			result.contacts[0] = pn.point;
			result.contacts[1] = p1.position - p1.radius * result.normal;
			return result;
		}
		else {
			return std::nullopt;
		}
	}
	std::optional<Collision> obb_particle_collide(const RigidBody& p0, const Particle& p1) {
		vec3_t tp = p0.to_local(p1.position);

		PointNorm pn = sdBox(tp, p0.dims);

		if (pn.distance < p1.radius) {
			Collision result;
			result.normal = p0.to_world_vector(pn.normal);
			result.contacts[0] = pn.point;
			result.contacts[1] = p1.position - p1.radius * result.normal;
			return result;
		}
		else {
			return std::nullopt;
		}
	}
	std::optional<Collision> sphere_particle_collide(const RigidBody& p0, const Particle& p1) {
		vec3_t tp = p0.to_local(p1.position);

		PointNorm pn = sdSphere(tp, p0.dims[0]);

		if (pn.distance < p1.radius) {
			Collision result;
			result.normal = p0.to_world_vector(pn.normal);
			result.contacts[0] = pn.point;
			result.contacts[1] = p1.position - p1.radius * result.normal;
			return result;
		}
		else {
			return std::nullopt;
		}
	}
}