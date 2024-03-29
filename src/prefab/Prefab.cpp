#include <pbd/prefab/Prefab.hpp>

#include <cppitertools/itertools.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	void Prefab::serialize(const Prefab& prefab, std::string& data) {
		uint64_t
			numParticles = prefab.num_particles(),
			numTrackers = prefab.num_trackers();

		ez::serialize::u64(numParticles, data);
		ez::serialize::u64(numTrackers, data);

		for (const auto & particle : prefab.particles) {
			PrefabParticle::serialize(particle, data);
		}
		for (const auto& tracker : prefab.trackers) {
			PrefabTracker::serialize(tracker, data);
		}

		ConstraintList::serialize(prefab.constraints, data);
	}
	const char* Prefab::deserialize(const char* first, const char* last, Prefab& prefab) {
		prefab.clear();
		uint64_t numParticles = 0, numTrackers = 0;

		first = ez::deserialize::u64(first, last, numParticles);
		first = ez::deserialize::u64(first, last, numTrackers);

		prefab.particles.reserve(numParticles);
		prefab.trackers.reserve(numTrackers);

		for (uint64_t i : iter::range(numParticles)) {
			PrefabParticle particle;

			first = PrefabParticle::deserialize(first, last, particle);

			prefab.particles.push_back(particle);
		}

		for (uint64_t i : iter::range(numTrackers)) {
			PrefabTracker tracker;

			first = PrefabTracker::deserialize(first, last, tracker);

			prefab.trackers.push_back(tracker);
		}

		return ConstraintList::deserialize(first, last, prefab.constraints);
	}


	Prefab::Error Prefab::validate() const noexcept {
		// Verify that the trackers and constraints are referring to something.
		if (particles.empty() && (!constraints.empty() || !trackers.empty())) {
			return Error::NoParticles;
		}

		// Range check all the constraints
		//for (ConstConstraintRef ref : constraints) {
			
		//}
	}

	bool Prefab::empty() const noexcept {
		return num_particles() == 0 && num_constraints() == 0 && num_trackers() == 0;
	}
	void Prefab::clear() {
		particles.clear();
		constraints.clear();
		trackers.clear();
	}

	size_t Prefab::num_particles() const noexcept {
		return particles.size();
	}
	size_t Prefab::num_constraints() const noexcept {
		return constraints.size();
	}
	size_t Prefab::num_trackers() const noexcept {
		return trackers.size();
	}

	int32_t Prefab::add_particle(const vec3_t& _pos, real_t _imass, real_t _radius, uint32_t _groups, uint32_t _mask) {
		particles.emplace_back(_pos, _imass, _radius, _groups, _mask);
		return static_cast<int32_t>(particles.size() - 1);
	}
	int32_t Prefab::add_particle(const vec3_t& _pos, const vec3_t& _vel, real_t _imass, real_t _radius, uint32_t _groups, uint32_t _mask) {
		particles.emplace_back(_pos, _vel, _imass, _radius, _groups, _mask);
		return static_cast<int32_t>(particles.size() - 1);
	}

	void Prefab::add_tracker(std::string_view _name, int32_t p0, int32_t p1, int32_t p2, int32_t p3) {
		trackers.emplace_back(_name, p0, p1, p2, p3);
	}
}