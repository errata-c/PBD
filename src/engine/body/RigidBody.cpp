#include <pbd/engine/body/RigidBody.hpp>



namespace pbd {
	BBox3 RigidBody::bounds() const noexcept {
		glm::vec3 min, max;

		switch (shape) {
		case Shape::Capsule:
			max.x = dims.x;
			max.y = dims.x;
			max.z = dims.y + dims.x;
			break;
		case Shape::Cylinder:
			max.x = dims.x;
			max.y = dims.x;
			max.z = dims.y;
			break;
		case Shape::OBB:
			max = dims;
			break;
		case Shape::Sphere:
			max = glm::vec3(dims.x);
			break;
		}
		min = -max;

		// Rotate the bounds?

		return BBox3(min, max);
	}
	BBox3 RigidBody::skewed_bounds(float delta) const noexcept {
		return {};
	}
	void RigidBody::calculate_inertia() {
		// Early exit for infinite mass.
		if (std::abs(imass) < 1e-5f) {
			inertia = glm::vec3(0.f);
			return;
		}

		float mass = 1.f / imass;
		switch (shape) {
		case Shape::Capsule: {
			static constexpr float pi = 3.14159265358979323846;
			float r2 = dims.x * dims.x;
			float r3 = dims.x * dims.x * dims.x;
			float h2 = dims.y * dims.y;

			float u = (dims.x * dims.x * dims.y * pi) / (((4.f / 3.f) * pi * r3) + (pi * r2 * dims.y));
			float mh = mass * (1.f -u);
			float mc = mass * u;

			inertia.x = mc * (h2 / 12.f + r2 / 4.f) + mh * ((2.f / 5.f) * r2 + h2 / 2.f + (3.f / 8.f) * dims.x * dims.y);
			inertia.y = inertia.x;
			inertia.z = mc  * (r2 / 2.f) + mh * ((2.f/ 5.f) * r2);
			break;
		}
		case Shape::Cylinder:
			inertia.z = 0.5f * mass * dims.x * dims.x;
			inertia.x = (mass * (3.f * dims.x * dims.x + dims.y * dims.y)) / 12.f;
			inertia.y = inertia.x;
			break;
		case Shape::OBB:
			inertia = glm::vec3(
				dims.x * dims.x + dims.y * dims.y,
				dims.y * dims.y + dims.z * dims.z,
				dims.x * dims.x + dims.z * dims.z
			) * (mass / 12.f);
			break;
		case Shape::Sphere:
			inertia = glm::vec3((2.f / 5.f) * mass * dims.x * dims.x);
			break;
		}

		inertia = glm::max(inertia, glm::vec3(1e-5f));
		inertia = glm::vec3(1.f) / inertia;
	}

	// Assume normalized orientation quaternion!
	glm::vec3 RigidBody::to_local_vector(const glm::vec3& v) const noexcept {
		glm::quat tmp(0.f, v.x, v.y, v.z);
		tmp = glm::conjugate(orientation) * tmp * orientation;
		return glm::vec3{ tmp.x, tmp.y, tmp.z };
	}
	glm::vec3 RigidBody::to_world_vector(const glm::vec3& v) const noexcept {
		glm::quat tmp(0.f, v.x, v.y, v.z);
		tmp = orientation * tmp * glm::conjugate(orientation);
		return glm::vec3{ tmp.x, tmp.y, tmp.z };
	}
	glm::vec3 RigidBody::to_local(const glm::vec3& v) const noexcept {
		return to_local_vector(v - position);
	}
	glm::vec3 RigidBody::to_world(const glm::vec3& v) const noexcept {
		return to_world_vector(v) + position;
	}
}