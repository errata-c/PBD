#include <pbd/engine/collider/BodyParticleCollision.hpp>

#include <pbd/engine/Particle.hpp>
#include <pbd/engine/RigidBody.hpp>
#include <pbd/common/Utils.hpp>

namespace pbd {
	PointNorm sdCapsule(const glm::vec3 & p, float r, float h) {
		glm::vec2 p2(p.x, p.y);
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
			result.point = glm::vec3(0.f, 0.f, glm::sign(p.z) * h * 0.5f);
			result.normal = p - result.point;
			result.distance = glm::length(result.normal);
			result.normal /= result.distance;

			result.point += result.normal * r;
			result.distance -= r;
		}

		return result;
	}
	PointNorm sdBox(const glm::vec3 & p, const glm::vec3 & b) {
		// Box centered on origin
		glm::vec3 q = glm::abs(p) - b;
		float mq = std::max(std::max(q.x, q.y), q.z);

		PointNorm result;

		if (mq > 0.f) {
			// Outside the box
			float d = glm::length(q);
			
			result.normal = q * glm::sign(p);
			result.distance = d;
			result.point = p - result.normal;
			result.normal /= result.distance;
			return result;
		}
		else {
			// Inside the box?
			int mi = 0;
			mq = q[0];
			for (int i = 1; i < 3; ++i) {
				if (q[i] > mq) {
					mq = q[i];
					mi = i;
				}
			}

			result.normal = glm::vec3(0);
			result.normal[mi] = glm::sign(p[mi]);
			result.distance = mq;
			result.point = p - result.normal * result.distance;
			return result;
		}
	}
	PointNorm sdSphere(const glm::vec3 & p, float r) {
		float l = glm::length(p);
		PointNorm result;
		result.normal = p / l;
		result.point = result.normal * r;
		result.distance = l - r;
		return result;
	}
	PointNorm sdCylinder(const glm::vec3 & p, float r, float h) {
		glm::vec2 p2(p.x, p.y);
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
			float d2 = glm::dot(p2, p2);
			if (d2 < r*r) {
				// Above
				result.normal = glm::vec3(0.f, 0.f, glm::sign(p.z));
				result.distance = std::abs(p.z) - h * 0.5f;
				result.point = p - result.normal * result.distance;
			}
			else {
				// ringed region
				float d = std::sqrt(d2);
				p2 /= d;
				p2 *= r;

				result.point = glm::vec3(p2.x, p2.y, glm::sign(p.z) * h * 0.5f);
				result.normal = p - result.point;
				result.distance = glm::length(result.normal);
				result.normal /= result.distance;
			}
		}

		return result;
	}

	std::optional<Collision> capsule_particle_collide(const RigidBody& p0, const Particle& p1) {
		glm::vec3 tp = p0.to_local(p1.position);
		
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
		glm::vec3 tp = p0.to_local(p1.position);

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
		glm::vec3 tp = p0.to_local(p1.position);

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
		glm::vec3 tp = p0.to_local(p1.position);

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