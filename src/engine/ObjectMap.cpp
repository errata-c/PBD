#include <pbd/engine/ObjectMap.hpp>
#include <cppitertools/itertools.hpp>

#include <algorithm>

namespace pbd {
	using const_iterator = ObjectMap::const_iterator;
	using iterator = ObjectMap::iterator;

	IndexRange::IndexRange(int32_t _first, int32_t _last) 
		: first(_first)
		, last(_last)
	{}
	int32_t IndexRange::size() const noexcept {
		return last - first;
	}
	bool IndexRange::empty() const noexcept {
		return first == last;
	}
	void IndexRange::offset(int32_t off) noexcept {
		first += off;
		last += off;
	}

	ObjectInfo::ObjectInfo(ObjectID _id, IndexRange p, IndexRange c, IndexRange t)
		: mid(_id)
		, mparticles(p)
		, mconstraints(c)
		, mtrackers(t)
	{}
	ObjectID ObjectInfo::id() const noexcept {
		return mid;
	}

	const IndexRange& ObjectInfo::particles() const noexcept {
		return mparticles;
	}
	const IndexRange& ObjectInfo::constraints() const noexcept {
		return mconstraints;
	}
	const IndexRange& ObjectInfo::trackers() const noexcept {
		return mtrackers;
	}
	IndexRange& ObjectInfo::particles() noexcept {
		return mparticles;
	}
	IndexRange& ObjectInfo::constraints() noexcept {
		return mconstraints;
	}
	IndexRange& ObjectInfo::trackers() noexcept {
		return mtrackers;
	}


	ObjectMap::ObjectMap()
		: rng(pcg_extras::seed_seq_from<std::random_device>{})
	{}

	ObjectID ObjectMap::create(const IndexRange& particles, const IndexRange& constraints, const IndexRange& trackers) {
		ObjectID id = generate_id();

		objects.emplace_back(
			id,
			particles,
			constraints,
			trackers);

		map[id] = objects.size()-1;

		return id;
	}

	void ObjectMap::destroy(const ObjectQueue & queue) {
		if (queue.empty()) {
			return;
		}

		iterator last = std::remove_if(objects.begin(), objects.end(), [&](const ObjectInfo& obj){ 
			return queue.contains(obj.id());
		});
		objects.erase(last, objects.end());

		map.clear();
		for (auto&& [i, obj] : iter::enumerate(objects)) {
			map[obj.id()] = i;
		}
	}

	void ObjectMap::clear() {
		map.clear();
		objects.clear();
	}
	size_t ObjectMap::size() const noexcept {
		return objects.size();
	}
	bool ObjectMap::empty() const noexcept {
		return objects.empty();
	}

	bool ObjectMap::contains(ObjectID id) const noexcept {
		return map.find(id) != map.end();
	}

	const_iterator ObjectMap::find(ObjectID id) const noexcept {
		auto it = map.find(id);
		if (it != map.end()) {
			return objects.begin() + it->second;
		}
		else {
			return end();
		}
	}

	iterator ObjectMap::begin() noexcept {
		return objects.begin();
	}
	iterator ObjectMap::end() noexcept {
		return objects.end();
	}

	const_iterator ObjectMap::begin() const noexcept {
		return objects.begin();
	}
	const_iterator ObjectMap::end() const noexcept {
		return objects.end();
	}

	ObjectID ObjectMap::generate_id() const {
		std::uniform_int_distribution<uint64_t> dist(0, ~uint64_t(0) - 1);
		ObjectID id;
		do {
			id = static_cast<ObjectID>(dist(rng));
		} while(contains(id));
		
		return id;
	}
}