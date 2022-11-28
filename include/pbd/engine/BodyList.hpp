#pragma once
#include <vector>
#include <cinttypes>
#include <pbd/common/Types.hpp>
#include <pbd/engine/RigidBody.hpp>

namespace pbd {
	class PrefabBody;

	class BodyList {
	public:
		using container_t = std::vector<RigidBody>;
		using iterator = container_t::iterator;
		using const_iterator = container_t::const_iterator;
		using reverse_iterator = container_t::reverse_iterator;
		using const_reverse_iterator = container_t::const_reverse_iterator;

		~BodyList() = default;
		BodyList() = default;
		BodyList(BodyList&&) noexcept = default;
		BodyList& operator=(BodyList&&) noexcept = default;
		BodyList(const BodyList&) = default;
		BodyList& operator=(const BodyList&) = default;

		size_t size() const noexcept;
		bool empty() const noexcept;

		int32_t add(
			Shape shape, 
			const Transform3 & form,
			const vec3_t & velocity, 
			const vec3_t & angular_velocity,
			real_t imass,
			const vec3_t & dims,
			uint32_t _group = 0u, 
			uint32_t _mask = 0u
		);
		int32_t add(
			const RigidBody & body
		);

		int32_t add_sphere(
			const Transform3& form, 
			real_t imass, 
			real_t radius, 
			const vec3_t& velocity = vec3_t(0.f),
			const vec3_t& angular_velocity = vec3_t(0.f),
			uint32_t _group = 0u,
			uint32_t _mask = 0u
		);
		int32_t add_capsule(
			const Transform3& form,
			real_t imass,
			real_t radius,
			real_t height,
			const vec3_t& velocity = vec3_t(0.f),
			const vec3_t& angular_velocity = vec3_t(0.f),
			uint32_t _group = 0u,
			uint32_t _mask = 0u
		);
		int32_t add_cylinder(
			const Transform3& form,
			real_t imass,
			real_t radius,
			real_t height,
			const vec3_t& velocity = vec3_t(0.f),
			const vec3_t& angular_velocity = vec3_t(0.f),
			uint32_t _group = 0u,
			uint32_t _mask = 0u
		);
		int32_t add_box(
			const Transform3& form,
			real_t imass,
			const vec3_t & dims,
			const vec3_t& velocity = vec3_t(0.f),
			const vec3_t& angular_velocity = vec3_t(0.f),
			uint32_t _group = 0u,
			uint32_t _mask = 0u
		);

		//int32_t add(const PrefabParticle& particle);
		//int32_t add(const PrefabParticle& particle, const Transform3& form);

		void shift(int32_t first, int32_t last, int32_t amount);
		void pop(int32_t amount);
		void clear();
		void reserve(int32_t amount);

		RigidBody& operator[](size_t i) noexcept;
		const RigidBody& operator[](size_t i) const noexcept;

		iterator begin() noexcept {
			return data.begin();
		}
		iterator end() noexcept {
			return data.end();
		}

		const_iterator begin() const noexcept {
			return data.begin();
		}
		const_iterator end() const noexcept {
			return data.end();
		}

		reverse_iterator rbegin() noexcept {
			return data.rbegin();
		}
		reverse_iterator rend() noexcept {
			return data.rend();
		}

		const_reverse_iterator rbegin() const noexcept {
			return data.rbegin();
		}
		const_reverse_iterator rend() const noexcept {
			return data.rend();
		}
	private:
		container_t data;
	};
}