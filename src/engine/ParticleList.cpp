#include <pbd/engine/ParticleList.hpp>
#include <pbd/prefab/PrefabParticle.hpp>
#include <limits>
#include <cassert>

namespace pbd {
	static constexpr size_t ParticleLimit = std::numeric_limits<int32_t>::max();

	int32_t ParticleList::add(const glm::vec3& _pos, const glm::vec3& _vel, float _imass, float _radius, uint32_t _group, uint32_t _mask) {
		assert(size() < ParticleLimit);
		int32_t id = static_cast<int32_t>(size());
		pos.push_back(_pos);
		prevPos.push_back(_pos);
		velocity.push_back(_vel);
		invMass.push_back(_imass);
		radius.push_back(_radius);
		force.push_back(glm::vec3(0));
		collision.push_back({_group, _mask});

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
		return pos.size();
	}
	bool ParticleList::empty() const noexcept {
		return pos.empty();
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

		shift_back(pos, first, last, amount);
		shift_back(prevPos, first, last, amount);
		shift_back(velocity, first, last, amount);
		shift_back(force, first, last, amount);
		shift_back(invMass, first, last, amount);
		shift_back(radius, first, last, amount);
		shift_back(collision, first, last, amount);
	}
	template<typename T>
	void erase_back(T& container, int32_t amount) {
		container.erase(container.end() - amount, container.end());
	}

	void ParticleList::pop(int32_t amount) {
		erase_back(pos, amount);
		erase_back(prevPos, amount);
		erase_back(velocity, amount);
		erase_back(force, amount);
		erase_back(invMass, amount);
		erase_back(radius, amount);
		erase_back(collision, amount);
	}

	void ParticleList::clear() {
		pos.clear();
		prevPos.clear();
		velocity.clear();
		force.clear();
		invMass.clear();
		radius.clear();
		collision.clear();
	}

	void ParticleList::reserve(int32_t amount) {
		assert(amount < ParticleLimit);

		pos.reserve(amount);
		prevPos.reserve(amount);
		velocity.reserve(amount);
		force.reserve(amount);
		invMass.reserve(amount);
		radius.reserve(amount);
		collision.reserve(amount);
	}
}