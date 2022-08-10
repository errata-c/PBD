#include <pbd/hash/Table.hpp>
#include <cmath>
#include <algorithm>

namespace pbd {
	using const_iterator = HashTable::const_iterator;
	using CellRange = HashTable::CellRange;

	static int32_t icoord(float val, float spacing) {
		return static_cast<int32_t>(val / spacing);
	}

	static int32_t hash(int32_t xi, int32_t yi, int32_t zi) {
		return (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481);
	}

	static int32_t hash(const glm::vec3& p, float spacing) {
		int32_t 
			xi = icoord(p.x, spacing), 
			yi = icoord(p.y, spacing),
			zi = icoord(p.z, spacing);

		return hash(xi, yi, zi);
	}

	HashTable::HashTable(float _spacing)
		: spacing(_spacing)
	{}
	HashTable::HashTable(float _spacing, int64_t initialCap)
		: spacing(_spacing)
	{
		cellEntries.reserve(initialCap);
	}

	void HashTable::clear() {
		cellMap.clear();
		cellEntries.clear();
	}
	
	void HashTable::build(const std::vector<int32_t>& ids, const std::vector<BBox>& boxes) {
		cellMap.clear();

		int64_t count = 0;
		for (const BBox & box: boxes) {
			int32_t
				x0 = icoord(box.min.x, spacing),
				y0 = icoord(box.min.y, spacing),
				z0 = icoord(box.min.z, spacing);

			int32_t
				x1 = icoord(box.max.x, spacing),
				y1 = icoord(box.max.y, spacing),
				z1 = icoord(box.max.z, spacing);

			// For all the cells the box covers, increment the value in the cell map.
			for (int32_t x = x0; x <= x1; ++x) {
				for (int32_t y = y0; y <= y1; ++y) {
					for (int32_t z = z0; z <= z1; ++z) {
						int32_t hv = hash(x, y, z);
						auto it = cellMap.find(hv);
						if (it == cellMap.end()) {
							// We start at 2, to reserve a place for the entry count.
							// We put it in the entry list so the elements in the cell map are as small as possible.
							cellMap.insert(it, { hv, 2 });
							count += 2;
						}
						else {
							++it->second;
							++count;
						}
					}
				}
			}
		}
		
		std::fill(cellEntries.begin(), cellEntries.end(), 0);
		if (cellEntries.size() < count) {
			cellEntries.resize(count, 0);
		}
		
		count = 0;
		for (auto & kv : cellMap) {
			int64_t tmp = kv.second;
			kv.second = count;
			count += tmp;
			
			// First entry is the number of ids in the cell.
			cellEntries[kv.second] = tmp;

			// Put the offset to the end of this cell's entry list at the first position in list.
			// We will use this in the next step to insert properly.
			cellEntries[kv.second+1] = tmp - 1;
		}

		for (int64_t i = 0, nboxes = boxes.size(); i < nboxes; ++i) {
			const BBox & box = boxes[i];
			int32_t id = ids[i];

			int32_t
				x0 = icoord(box.min.x, spacing),
				y0 = icoord(box.min.y, spacing),
				z0 = icoord(box.min.z, spacing);

			int32_t
				x1 = icoord(box.max.x, spacing),
				y1 = icoord(box.max.y, spacing),
				z1 = icoord(box.max.z, spacing);

			// For all the cells the box covers, insert the id into the list of entries.
			for (int32_t x = x0; x <= x1; ++x) {
				for (int32_t y = y0; y <= y1; ++y) {
					for (int32_t z = z0; z <= z1; ++z) {
						int32_t hv = hash(x, y, z);
						int32_t start = cellMap[hv];
						
						int32_t& offset = cellEntries[start+1];
						cellEntries[start+offset+1] = id;
						--offset;
					}
				}
			}
		}

		// FIN!
	}

	int64_t HashTable::size() const noexcept {
		return static_cast<int64_t>(cellMap.size());
	}


	
	const_iterator HashTable::begin() const noexcept {
		return const_iterator(cellMap.begin(), cellEntries);
	}
	const_iterator HashTable::end() const noexcept {
		return const_iterator(cellMap.begin(), cellEntries);
	}
	


	CellRange::CellRange()
		: first(nullptr)
		, last(nullptr)
	{}
	CellRange::CellRange(const std::vector<int32_t>& list, int32_t index)
	{
		first = list.data() + index;
		last = first + (*first);

		++first;
		++last;
	}
	int32_t CellRange::size() const noexcept {
		return last - first;
	}
	bool CellRange::empty() const noexcept {
		return first == last;
	}
	const int32_t& CellRange::operator[](int32_t i) const noexcept {
		assert(i >= 0 && i < size());
		return first[i];
	}
	const int32_t* CellRange::begin() const noexcept {
		return first;
	}
	const int32_t* CellRange::end() const noexcept {
		return last;
	}



	const_iterator::const_iterator(const _map_iter& ref, const std::vector<int32_t>& e)
		: it(ref)
		, entries(&e)
	{}
	CellRange const_iterator::operator*() const {
		return CellRange(*entries, it->second);
	}
	CellRange const_iterator::operator->() const {
		return CellRange(*entries, it->second);
	}
	const_iterator const_iterator::operator++(int) {
		const_iterator copy = *this;
		++it;
		return copy;
	}
	const_iterator& const_iterator::operator++() {
		++it;
		return *this;
	}
	bool const_iterator::operator!=(const const_iterator& other) const noexcept {
		return it != other.it;
	}
	bool const_iterator::operator==(const const_iterator& other) const noexcept {
		return it == other.it;
	}
}