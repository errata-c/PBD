#pragma once
#include <string>
#include <cinttypes>
#include <vector>
#include <pbd/common/TransformTracker.hpp>

namespace pbd {
	class PrefabTracker;

	class TrackerList {
	public:
		using container_t = std::vector<TransformTracker>;
		using iterator = container_t::iterator;
		using const_iterator = container_t::const_iterator;

		size_t size() const noexcept;
		bool empty() const noexcept;

		void shift(int32_t first, int32_t last, int32_t amount);

		void add(int i0, int i1, int i2, int i3, const ParticleList& engine);
		void add(const PrefabTracker & tracker, const ParticleList& engine, int32_t offset);

		void pop(int32_t count);

		iterator begin() noexcept;
		iterator end() noexcept;

		const_iterator begin() const noexcept;
		const_iterator end() const noexcept;
	private:
		std::vector<TransformTracker> trackers;
	};
}