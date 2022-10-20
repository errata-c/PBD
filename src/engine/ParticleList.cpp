#include <pbd/engine/ParticleList.hpp>
#include <pbd/prefab/PrefabParticle.hpp>
#include <limits>
#include <cassert>

namespace pbd {
	static constexpr size_t ParticleLimit = std::numeric_limits<int32_t>::max();

	int32_t ParticleList::add(const glm::vec3& _pos, const glm::vec3& _vel, float _imass, float _radius, uint32_t _group, uint32_t _mask) {
		assert(size() < ParticleLimit);
		int32_t id = static_cast<int32_t>(size());
		data.push_back(Particle{_pos, _vel, _imass, _radius, _group, _mask});

		return id;
	}
	int32_t ParticleList::add(const glm::vec3& _pos, float _imass, float _radius, uint32_t _group, uint32_t _mask) {
		return add(_pos, glm::vec3(0.f), _imass, _radius, _group, _mask);
	}
	int32_t ParticleList::add(const PrefabParticle& particle) {
		return add(particle.position, particle.velocity, particle.imass, particle.radius, particle.collision_groups, particle.collision_mask);
	}
	int32_t ParticleList::add(const PrefabParticle& particle, const Transform3& form) {
		return add(
			form.toWorld(particle.position), 
			form.toWorldVector(particle.velocity), 
			particle.imass / form.size, 
			particle.radius * form.size,
			particle.collision_groups,
			particle.collision_mask
		);
	}

	size_t ParticleList::size() const noexcept {
		return data.size();
	}
	bool ParticleList::empty() const noexcept {
		return data.empty();
	}

	template<typename T>
	void shift_back(T& container, int32_t first, int32_t last, int32_t amount) {
		auto rpos = container.begin() + first;
		auto rlast = container.begin() + last;
		auto wpos = rpos + amount;
		while (rpos != rlast) {
			*wpos++ = *rpos++;
		}
	}

	void ParticleList::shift(int32_t first, int32_t last, int32_t amount) {
		assert(first <= last);
		assert(first + amount >= 0);
		assert(last + amount < size());

		shift_back(data, first, last, amount);
	}
	template<typename T>
	void erase_back(T& container, int32_t amount) {
		container.erase(container.end() - amount, container.end());
	}

	void ParticleList::pop(int32_t amount) {
		erase_back(data, amount);
	}

	void ParticleList::clear() {
		data.clear();
	}

	void ParticleList::reserve(int32_t amount) {
		assert(amount < ParticleLimit);

		data.reserve(amount);
	}

	particle_span ParticleList::get_particles(int32_t first, int32_t last) {
		assert(first < last);
		assert(last < data.size());
		assert(first >= 0);

		return particle_span{&data[first], static_cast<size_t>(last - first) };
	}

	Particle& ParticleList::operator[](size_t i) noexcept {
		return data[i];
	}
	const Particle& ParticleList::operator[](size_t i) const noexcept {
		return data[i];
	}
}