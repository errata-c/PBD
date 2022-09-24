#include <pbd/engine/ObjectQueue.hpp>

namespace pbd {
	size_t ObjectQueue::size() const noexcept {
		return dqueue.size();
	}
	bool ObjectQueue::empty() const noexcept {
		return dqueue.empty();
	}
	bool ObjectQueue::contains(ObjectID id) const noexcept {
		return dqueue.contains(id);
	}

	bool ObjectQueue::push(ObjectID id) {
		auto it = dqueue.insert(id);
		return it.second;
	}
	void ObjectQueue::clear() {
		dqueue.clear();
	}
}