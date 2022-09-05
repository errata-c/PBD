#include <pbd/common/ConstraintList.hpp>
#include <cassert>

namespace pbd {
	ConstraintRef ConstraintList::operator[](int64_t i) {
		assert(i >= 0 && i < size());

		const CVariant & cvar = constraints[i];
		return ConstraintRef(cvar.kind, cdata.data() + cvar.index);
	}
	ConstConstraintRef ConstraintList::operator[](int64_t i) const {
		assert(i >= 0 && i < size());

		const CVariant& cvar = constraints[i];
		return ConstConstraintRef(cvar.kind, cdata.data() + cvar.index);
	}

	
	void ConstraintList::erase(int64_t i) {
		assert(i >= 0 && i < size());

		const CVariant & cvar = constraints[i];
		size_t siz = SizeOf(cvar.kind);

		auto it = cdata.begin() + cvar.index;
		cdata.erase(it, it + siz / sizeof(int32_t));
		constraints.erase(constraints.begin() + i);
	}
	void ConstraintList::erase(int64_t first, int64_t last) {
		assert(first >= 0 && first < size());
		assert(last >= first && last <= size());

		if (first == last) {
			return;
		}

		int64_t start = constraints[first].index;
		int64_t offset = 0;

		for (int64_t i = first; i < last; ++i) {
			offset += SizeOf(constraints[i].kind);
		}

		{
			auto it = cdata.begin() + start;
			cdata.erase(it, it + offset);
		}

		constraints.erase(constraints.begin() + first, constraints.begin() + last);
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
		for (size_t i = 0, count = other.size(); i < count; ++i) {
			ConstConstraintRef ref = other[i];
			constraints.push_back(CVariant{ref.type(), static_cast<int64_t>(cdata.size())});

			size_t u = 0, ucount = NumIds(ref.type());
			for (; u < ucount; ++u) {
				cdata.push_back(ref.data()[u] + offset);
			}
			ucount = SizeOf(ref.type());
			for (; u < ucount; ++u) {
				cdata.push_back(ref.data()[u]);
			}
		}
	}
}