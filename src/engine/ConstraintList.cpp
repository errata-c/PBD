#include <pbd/engine/ConstraintList.hpp>

namespace pbd {
	ConstraintRef ConstraintList::operator[](int64_t i) {
		
	}
	ConstConstraintRef ConstraintList::operator[](int64_t i) const {
		
	}

	
	void ConstraintList::erase(int64_t i) {

	}
	void ConstraintList::erase(int64_t first, int64_t last) {

	}

	int64_t ConstraintList::size() const noexcept {
		return static_cast<int64_t>(constraints.size());
	}
	bool ConstraintList::empty() const noexcept {
		return constraints.empty();
	}

	void ConstraintList::clear() {
		constraints.clear();
		cdata.clear();
	}
	void ConstraintList::append(const ConstraintList& other, int32_t offset) {
		
	}
}