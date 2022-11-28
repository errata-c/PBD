#include <pbd/engine/RigidBody.hpp>
#include <pbd/common/Utils.hpp>

namespace pbd {
	RigidBody::RigidBody()
		: RigidBody(Shape::Sphere, Transform3{}, 1.0, vec3_t(1,0,0))
	{}
	RigidBody::RigidBody(
		Shape _shape,
		const Transform3& _form,
		real_t _imass,
		const vec3_t& _dims,
		uint32_t _cgroups,
		uint32_t _cmask
	) : RigidBody(_shape, _form.origin, _form.rotation, _imass, _dims, vec3_t(0), vec3_t(0), _cgroups, _cmask)
	{}
	RigidBody::RigidBody(
		Shape _shape,
		const vec3_t& _position,
		const quat_t& _orientation,
		real_t _imass,
		const vec3_t& _dims,
		uint32_t _cgroups,
		uint32_t _cmask
	) : RigidBody(_shape, _position, _orientation, _imass, _dims, vec3_t(0), vec3_t(0), _cgroups, _cmask)
	{}
	RigidBody::RigidBody(
		Shape _shape,
		const Transform3& _form,
		real_t _imass,
		const vec3_t& _dims,
		const vec3_t& _velocity,
		const vec3_t& _angular_velocity,
		uint32_t _cgroups,
		uint32_t _cmask 
	) : RigidBody(_shape, _form.origin, _form.rotation, _imass, _dims, _velocity, _angular_velocity, _cgroups, _cmask)
	{}
	RigidBody::RigidBody(
		Shape _shape,
		const vec3_t& _position,
		const quat_t& _orientation,
		real_t _imass,
		const vec3_t& _dims,
		const vec3_t& _velocity,
		const vec3_t& _angular_velocity,
		uint32_t _cgroups,
		uint32_t _cmask
	)
		: collision_groups(_cgroups)
		, collision_mask(_cmask)
		, position(_position)
		, velocity(_velocity)
		, imass(_imass)

		, dims(_dims)
		, orientation(_orientation)
		, angular_velocity(_angular_velocity)
		, shape(_shape)
	{
		// inverse mass CANNOT be negative
		assert(imass >= real_t(0));

		// Dims CANNOT be negative
		assert(dims[0] >= real_t(0));
		assert(dims[1] >= real_t(0));
		assert(dims[2] >= real_t(0));

		calculate_inertia();
	}

	void RigidBody::set_mass(real_t mass) noexcept {
		set_imass(1.0 / std::max(mass, eps()));
	}
	void RigidBody::set_imass(real_t _imass) noexcept {
		imass = _imass;
		calculate_inertia();
	}

	real_t& RigidBody::width() noexcept {
		return dims[0];
	}
	const real_t& RigidBody::width() const noexcept {
		return dims[0];
	}
	real_t& RigidBody::radius() noexcept {
		return dims[0];
	}
	const real_t& RigidBody::radius() const noexcept {
		return dims[0];
	}
	real_t& RigidBody::height() noexcept {
		return dims[1];
	}
	const real_t& RigidBody::height() const noexcept {
		return dims[1];
	}
	real_t& RigidBody::depth() noexcept {
		return dims[2];
	}
	const real_t& RigidBody::depth() const noexcept {
		return dims[2];
	}

	BBox3 RigidBody::bounds() const noexcept {
		vec3_t min, max;

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
			max = vec3_t(dims.x);
			break;
		}
		min = -max;

		// Rotate the bounds?

		return BBox3(min, max);
	}
	BBox3 RigidBody::skewed_bounds(real_t delta) const noexcept {
		return {};
	}
	void RigidBody::calculate_inertia() {
		// Early exit for infinite mass.
		if (std::abs(imass) < eps()) {
			inverse_inertia = vec3_t(0.f);
			return;
		}

		real_t mass = 1.f / imass;
		vec3_t inertia(0.f);
		switch (shape) {
		case Shape::Capsule: {
			static constexpr real_t pi = 3.14159265358979323846;
			real_t r2 = dims.x * dims.x;
			real_t r3 = dims.x * dims.x * dims.x;
			real_t h2 = dims.y * dims.y;

			real_t u = (dims.x * dims.x * dims.y * pi) / (((4.f / 3.f) * pi * r3) + (pi * r2 * dims.y));
			real_t mh = mass * (1.f -u);
			real_t mc = mass * u;

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
			inertia = vec3_t(
				dims.x * dims.x + dims.y * dims.y,
				dims.y * dims.y + dims.z * dims.z,
				dims.x * dims.x + dims.z * dims.z
			) * (mass / 12.f);
			break;
		case Shape::Sphere:
			inertia = vec3_t((2.f / 5.f) * mass * dims.x * dims.x);
			break;
		default:
			assert(false); // Forgot to implement a shape if you get this assertion!
			break;
		}

		// In theory, the inertia calculated here will always be a non-zero, positive number
		//inertia = glm::max(inertia, vec3_t(1e-5f));

		inverse_inertia = vec3_t(1.f) / inertia;
	}

	// Assume normalized orientation quaternion!
	vec3_t RigidBody::to_local_vector(const vec3_t& v) const noexcept {
		return pbd::reverse_rotate(orientation, v);
	}
	vec3_t RigidBody::to_world_vector(const vec3_t& v) const noexcept {
		return pbd::rotate(orientation, v);
	}
	vec3_t RigidBody::to_local(const vec3_t& v) const noexcept {
		return to_local_vector(v - position);
	}
	vec3_t RigidBody::to_world(const vec3_t& v) const noexcept {
		return to_world_vector(v) + position;
	}

	Transform3 RigidBody::transform() const noexcept {
		return Transform3(position, orientation);
	}
}