#include <pbd/engine/constraint/ConstraintList.hpp>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

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

	ConstraintList::iterator ConstraintList::begin() noexcept {
		return iterator(cdata.data(), constraints.begin());
	}
	ConstraintList::iterator ConstraintList::end() noexcept {
		return iterator(cdata.data(), constraints.end());
	}
	ConstraintList::const_iterator ConstraintList::begin() const noexcept {
		return const_iterator(cdata.data(), constraints.begin());
	}
	ConstraintList::const_iterator ConstraintList::end() const noexcept {
		return const_iterator(cdata.data(), constraints.end());
	}

	void ConstraintList::shift(int64_t first, int64_t last, int64_t amount) {
		assert(first >= 0);
		assert(last < size());
		assert(first  + amount >= 0);
		assert(last + amount < size());

		auto cfirst = constraints.begin() + first;
		auto clast = constraints.begin() + last;
		auto cwrite = cfirst + amount;

		// The amount the data is going to be shifted over is different from the amount parameter.
		// Find the first position we are shifting to, then calculate a new delta
		int64_t data_first = cfirst->index;
		int64_t data_last = (clast-1)->index + SizeOf((clast-1)->kind);

		auto dfirst = cdata.begin() + data_first;
		auto dlast = cdata.begin() + data_last;

		auto dwrite = cdata.begin();
		// We need to make sure we are not going to access out of bounds!
		if (cwrite != constraints.begin()) {
			// If cwrite-1 exists, then it MUST be a valid constraint.
			// cwrite itself might NOT be valid, which is why we have to check.
			dwrite + (cwrite-1)->index + SizeOf((cwrite-1)->kind);
		}

		// The amount the data is being shifted by.
		int64_t data_delta = dwrite - dfirst;

		// No need to update any references here, we just shift the data down first.
		while (dfirst != dlast) {
			*dwrite++ = *dfirst++;
		}

		// We must update the indices accordingly.
		while (cfirst != clast) {
			*cwrite = *cfirst++;
			cwrite->index += data_delta;
		}

		// We do NOT have to erase the old data, this method is meant to be used in a special external loop that will perform cleanup.
	}
	void ConstraintList::pop(int64_t count) {
		assert(count >= 0);
		assert(count <= size());

		if (count == 0) {
			return;
		}
		else if (count == size()) {
			cdata.clear();
			constraints.clear();
			return;
		}

		// Remove count elements from the back of the container.
		int64_t data_first = (constraints.end() - count)->index;
		cdata.erase(cdata.begin() + data_first, cdata.end());
		constraints.erase(constraints.end() - count, constraints.end());
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
	void ConstraintList::append(const ConstraintList& other, int32_t offset, const Transform3& form) {
		int64_t first = size();
		append(other, offset);
		int64_t last = size();
		for (; first != last; ++first) {
			(*this)[first].transform(form);
		}
	}


	void ConstraintList::serialize(const ConstraintList& clist, std::string& output) {
		output.reserve(output.size() + clist.cdata.size() * sizeof(int32_t) + clist.constraints.size() * sizeof(CVariant));

		uint64_t numConstraints = clist.constraints.size();
		uint64_t dataSize = clist.cdata.size();

		ez::serialize::u64(numConstraints, output);
		ez::serialize::u64(dataSize, output);

		for (uint64_t i = 0; i < numConstraints; ++i) {
			ez::serialize::enumerator(clist.constraints[i].kind, output);
			ez::serialize::i64(clist.constraints[i].index, output);
		}
		
		for (uint64_t i = 0; i < dataSize; ++i) {
			ez::serialize::i32(clist.cdata[i], output);
		}
	}
	const char* ConstraintList::deserialize(const char* first, const char* last, ConstraintList& clist) {
		uint64_t numConstraints = 0;
		uint64_t dataSize = 0;

		first = ez::deserialize::u64(first, last, numConstraints);
		first = ez::deserialize::u64(first, last, dataSize);

		clist.constraints.reserve(numConstraints);
		clist.cdata.reserve(dataSize);

		for (uint64_t i = 0; i < numConstraints; ++i) {
			CVariant cvar;

			first = ez::deserialize::enumerator(first, last, cvar.kind);
			first = ez::deserialize::i64(first, last, cvar.index);
			clist.constraints.push_back(cvar);
		}

		for (uint64_t i = 0; i < dataSize; ++i) {
			int32_t val;
			first = ez::deserialize::i32(first, last, val);
			clist.cdata.push_back(val);
		}

		return first;
	}
}