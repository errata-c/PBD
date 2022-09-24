#pragma once
#include <pbd/common/Types.hpp>

#include <vector>
#include <random>
#include <pcg_random.hpp>
#include <parallel_hashmap/phmap.h>

#include <pbd/engine/ObjectQueue.hpp>

namespace pbd {
	// The range of particles and constraints this object is defined by.
	struct IndexRange {
		IndexRange(int32_t _first = 0, int32_t _last = 0);

		int32_t first, last;

		int32_t size() const noexcept;
		bool empty() const noexcept;
		void offset(int32_t off) noexcept;
	};

	struct ObjectInfo {
		ObjectInfo(ObjectID _id, IndexRange p, IndexRange c, IndexRange t);

		ObjectID id() const noexcept;

		const IndexRange& particles() const noexcept;
		const IndexRange& constraints() const noexcept;
		const IndexRange& trackers() const noexcept;
	private:
		friend class ObjectMap;

		IndexRange& particles() noexcept;
		IndexRange& constraints() noexcept;
		IndexRange& trackers() noexcept;

		ObjectID mid;
		IndexRange mparticles, mconstraints, mtrackers;
	};

	class ObjectMap {
	public:
		using container_t = std::vector<ObjectInfo>;
		using iterator = container_t::iterator;
		using const_iterator = container_t::const_iterator;

		ObjectMap(ObjectMap &&) noexcept = default;
		ObjectMap & operator=(ObjectMap&&) noexcept = default;

		ObjectMap();
		
		ObjectID create(const IndexRange& particles, const IndexRange& constraints, const IndexRange& trackers);
		void destroy(const ObjectQueue& queue);

		void clear();
		size_t size() const noexcept;
		bool empty() const noexcept;

		bool contains(ObjectID id) const noexcept;

		const ObjectInfo& front() const;
		const ObjectInfo& back() const;

		const_iterator find(ObjectID id) const noexcept;

		iterator begin() noexcept;
		iterator end() noexcept;

		const_iterator begin() const noexcept;
		const_iterator end() const noexcept;
	private:
		ObjectID generate_id() const;

		mutable pcg64 rng;
		phmap::flat_hash_map<ObjectID, ptrdiff_t> map;
		std::vector<ObjectInfo> objects;
	};
}