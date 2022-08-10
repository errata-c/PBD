#pragma once
#include <cinttypes>
#include <array>
#include <vector>
#include <glm/vec3.hpp>
#include <parallel_hashmap/phmap.h>

#include <pbd/hash/BBox.hpp>

namespace pbd {
	class HashTable {
	public:
		class const_iterator;

		HashTable(HashTable&&) noexcept = default;
		HashTable& operator=(HashTable&&) noexcept = default;

		HashTable(float _spacing);
		HashTable(float _spacing, int64_t initialCap);

		// Clear the table completely.
		void clear();

		// Insert a list of IDs given their bounding boxes.
		void build(const std::vector<int32_t> & ids, const std::vector<BBox> & boxes);

		int64_t size() const noexcept;

		const_iterator begin() const noexcept;
		const_iterator end() const noexcept;
	private:
		float spacing;
		phmap::parallel_flat_hash_map<int32_t, int32_t> cellMap;
		std::vector<int32_t> cellEntries;

	public:
		class CellRange {
		public:
			CellRange(const CellRange&) = default;
			CellRange& operator=(const CellRange&) = default;

			CellRange();
			CellRange(const std::vector<int32_t> & list, int32_t index);

			int32_t size() const noexcept;
			bool empty() const noexcept;

			const int32_t & operator[](int32_t i) const noexcept;

			const int32_t* begin() const noexcept;
			const int32_t* end() const noexcept;
		private:
			const int32_t *first, *last;
		};

		class const_iterator {
		public:
			using _map_iter = phmap::parallel_flat_hash_map<int32_t, int32_t>::const_iterator;

			const_iterator() = default;
			const_iterator(const const_iterator&) = default;
			const_iterator& operator=(const const_iterator&) = default;

			const_iterator(const _map_iter & ref, const std::vector<int32_t> & e);

			CellRange operator*() const;
			CellRange operator->() const;

			const_iterator operator++(int);
			const_iterator& operator++();

			bool operator!=(const const_iterator& other) const noexcept;
			bool operator==(const const_iterator& other) const noexcept;
		private:
			_map_iter it;
			const std::vector<int32_t> * entries;
		};
	};
}