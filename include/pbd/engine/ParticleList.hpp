#pragma once
#include <vector>
#include <cinttypes>
#include <glm/vec3.hpp>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>
#include <pbd/common/Span.hpp>

#include <pbd/engine/Particle.hpp>

namespace pbd {
	class PrefabParticle;

	class ParticleList {
	public:
		using container_t = std::vector<Particle>;
		using iterator = container_t::iterator;
		using const_iterator = container_t::const_iterator;
		using reverse_iterator = container_t::reverse_iterator;
		using const_reverse_iterator = container_t::const_reverse_iterator;

		~ParticleList() = default;
		ParticleList() = default;
		ParticleList(ParticleList&&) noexcept = default;
		ParticleList& operator=(ParticleList&&) noexcept = default;
		ParticleList(const ParticleList&) = default;
		ParticleList& operator=(const ParticleList& ) = default;

		size_t size() const noexcept;
		bool empty() const noexcept;

		int32_t add(const glm::vec3& position, const glm::vec3& velocity, float invMass, float radius, uint32_t _group = 0u, uint32_t _mask = 0u);
		int32_t add(const glm::vec3& position, float invMass, float radius, uint32_t _group = 0u, uint32_t _mask = 0u);
		int32_t add(const PrefabParticle& particle);
		int32_t add(const PrefabParticle& particle, const Transform3& form);

		void shift(int32_t first, int32_t last, int32_t amount);
		void pop(int32_t amount);
		void clear();
		void reserve(int32_t amount);


		Particle& operator[](size_t i) noexcept;
		const Particle& operator[](size_t i) const noexcept;

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