#include <pbd/engine/TrackerList.hpp>
#include <pbd/prefab/PrefabTracker.hpp>

namespace pbd {
	using iterator = TrackerList::iterator;
	using const_iterator = TrackerList::const_iterator;

	size_t TrackerList::size() const noexcept {
		return trackers.size();
	}
	bool TrackerList::empty() const noexcept {
		return trackers.empty();
	}

	void TrackerList::shift(int32_t first, int32_t last, int32_t amount) {
		auto rit = trackers.begin() + first;
		auto rlast = trackers.begin() + last;
		auto write = rit + amount;

		while (rit != rlast) {
			*write++ = *rit++;
		}
	}

	void TrackerList::add(int i0, int i1, int i2, int i3, const ParticleList& particles) {
		trackers.emplace_back(i0, i1, i2, i3, particles);
	}
	void TrackerList::add(const PrefabTracker& tracker, const ParticleList& particles, int32_t offset) {
		add(
			tracker.particles[0] + offset, 
			tracker.particles[1] + offset,
			tracker.particles[2] + offset,
			tracker.particles[3] + offset,
			particles);
	}

	void TrackerList::pop(int32_t count) {
		trackers.erase(trackers.end() - count, trackers.end());
	}

	iterator TrackerList::begin() noexcept {
		return trackers.begin();
	}
	iterator TrackerList::end() noexcept {
		return trackers.end();
	}

	const_iterator TrackerList::begin() const noexcept {
		return trackers.begin();
	}
	const_iterator TrackerList::end() const noexcept {
		return trackers.end();
	}
}