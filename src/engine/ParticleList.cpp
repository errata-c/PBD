#include <pbd/engine/ParticleList.hpp>
#include <pbd/prefab/PrefabParticle.hpp>
#include <limits>
#include <cassert>

namespace pbd {
	static constexpr size_t ParticleLimit = std::numeric_limits<int32_t>::max();

	int32_t ParticleList::add(const vec3_t& _pos, const vec3_t& _vel, real_t _imass, real_t _radius, uint32_t _group, uint32_t _mask) {
		assert(size() < ParticleLimit);
		int32_t id = static_cast<int32_t>(size());
		data.push_back(Particle{ _group, _mask, _pos, _vel, _imass, _radius, });

		return id;
	}
	int32_t ParticleList::add(const vec3_t& _pos, real_t _imass, real_t _radius, uint32_t _group, uint32_t _mask) {
		return add(_pos, vec3_t(0.f), _imass, _radius, _group, _mask);
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

	Particle& ParticleList::operator[](size_t i) noexcept {
		return data[i];
	}
	const Particle& ParticleList::operator[](size_t i) const noexcept {
		return data[i];
	}
}