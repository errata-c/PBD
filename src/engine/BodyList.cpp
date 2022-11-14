#include <pbd/engine/BodyList.hpp>

namespace pbd {
	size_t BodyList::size() const noexcept {
		return data.size();
	}
	bool BodyList::empty() const noexcept {
		return data.empty();
	}

	int32_t BodyList::add(
		Shape shape,
		const Transform3& form,
		const glm::vec3& velocity,
		const glm::vec3& angular_velocity,
		float imass,
		const glm::vec3& dims,
		uint32_t _group,
		uint32_t _mask
	) {
		return add(RigidBody{
			_group,
			_mask,
			
			form.origin,
			velocity,
			imass,
			dims,

			form.rotation,
			angular_velocity,
			glm::vec3(0),
			Shape::Sphere,
		});
	}
	int32_t BodyList::add(
		const RigidBody& body
	) {
		int32_t i = static_cast<int32_t>(data.size());
		data.push_back(body);
		data.back().calculate_inertia();
		return i;
	}

	int32_t BodyList::add_sphere(
		const Transform3& form,
		float imass,
		float radius,
		const glm::vec3& velocity,
		const glm::vec3& angular_velocity,
		uint32_t _group,
		uint32_t _mask
	) {
		return add(RigidBody{
			_group,
			_mask,
			
			form.origin, 
			velocity, 
			imass, 
			glm::vec3(radius),

			form.rotation, 
			angular_velocity, 
			glm::vec3(0), 
			Shape::Sphere,
		});
	}
	int32_t BodyList::add_capsule(
		const Transform3& form,
		float imass,
		float radius,
		float height,
		const glm::vec3& velocity,
		const glm::vec3& angular_velocity,
		uint32_t _group,
		uint32_t _mask
	) {
		return add(RigidBody{
			_group,
			_mask,
			
			form.origin,
			velocity,
			imass,
			glm::vec3(radius, height, height),
		
			form.rotation,
			angular_velocity,
			glm::vec3(0),
			Shape::Capsule,
		});
	}
	int32_t BodyList::add_cylinder(
		const Transform3& form,
		float imass,
		float radius,
		float height,
		const glm::vec3& velocity,
		const glm::vec3& angular_velocity,
		uint32_t _group,
		uint32_t _mask
	) {
		return add(RigidBody{
			_group,
			_mask,

			form.origin,
			velocity,
			imass,
			glm::vec3(radius, height, height),

			form.rotation,
			angular_velocity,
			glm::vec3(0),
			Shape::Cylinder,
			});
	}
	int32_t BodyList::add_box(
		const Transform3& form,
		float imass,
		const glm::vec3& dims,
		const glm::vec3& velocity,
		const glm::vec3& angular_velocity,
		uint32_t _group,
		uint32_t _mask
	) {
		return add(RigidBody{
			_group,
			_mask,

			form.origin,
			velocity,
			imass,
			dims,

			form.rotation,
			angular_velocity,
			glm::vec3(0),
			Shape::OBB,
		});
	}

	//int32_t add(const PrefabParticle& particle);
	//int32_t add(const PrefabParticle& particle, const Transform3& form);

	template<typename T>
	void shift_back(T& container, int32_t first, int32_t last, int32_t amount) {
		auto rpos = container.begin() + first;
		auto rlast = container.begin() + last;
		auto wpos = rpos + amount;
		while (rpos != rlast) {
			*wpos++ = *rpos++;
		}
	}
	template<typename T>
	void erase_back(T& container, int32_t amount) {
		container.erase(container.end() - amount, container.end());
	}

	void BodyList::shift(int32_t first, int32_t last, int32_t amount) {
		assert(first <= last);
		assert(first + amount >= 0);
		assert(last + amount < size());

		shift_back(data, first, last, amount);
	}
	void BodyList::pop(int32_t amount) {
		erase_back(data, amount);
	}
	void BodyList::clear() {
		data.clear();
	}
	void BodyList::reserve(int32_t amount) {
		data.reserve(amount);
	}

	RigidBody& BodyList::operator[](size_t i) noexcept {
		return data[i];
	}
	const RigidBody& BodyList::operator[](size_t i) const noexcept {
		return data[i];
	}
}