#pragma once
#include <pbd/common/Types.hpp>
#include <parallel_hashmap/phmap.h>

namespace pbd {
	class ObjectQueue {
	public:
		ObjectQueue() = default;
		~ObjectQueue() = default;
		ObjectQueue(ObjectQueue &&) noexcept = default;
		ObjectQueue& operator=(ObjectQueue&&) noexcept = default;
		ObjectQueue(const ObjectQueue&) = default;
		ObjectQueue& operator=(const ObjectQueue&) = default;

		size_t size() const noexcept;
		bool empty() const noexcept;
		bool contains(ObjectID id) const noexcept;

		bool push(ObjectID id);
		void clear();
	private:
		phmap::flat_hash_set<ObjectID> dqueue;
	};
}