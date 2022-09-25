#pragma once
#include <vector>
#include <string>
#include <cinttypes>

#include <pbd/common/Transform.hpp>

#include <pbd/engine/collide/Particle.hpp>
#include <pbd/engine/collide/Plane.hpp>

#include <pbd/engine/constraint/Distance.hpp>
#include <pbd/engine/constraint/TetraVolume.hpp>
#include <pbd/engine/constraint/NHTetraVolume.hpp>

#include <pbd/engine/ConstraintRef.hpp>

namespace pbd {
	class Engine;

	class ConstraintList {
	public:
		static void serialize(const ConstraintList & clist, std::string& output);
		static const char* deserialize(const char * first, const char * last, ConstraintList & clist);

		class iterator;
		class const_iterator;

		ConstraintList() = default;
		ConstraintList(const ConstraintList&) = default;
		ConstraintList& operator=(const ConstraintList &) = default;
		ConstraintList(ConstraintList&&) noexcept = default;
		ConstraintList& operator=(ConstraintList&&) noexcept = default;

		template<typename T>
		int64_t add(const T& cval) {
			int64_t index = cdata.size();
			Constraint kind = T::Kind;

			const int32_t* ival = (const int32_t*)&cval;
			for (size_t i = 0; i < (sizeof(T) / sizeof(int32_t)); ++i) {
				cdata.push_back(ival[i]);
			}
			constraints.push_back(CVariant{ kind, index });

			return constraints.size() - 1;
		}

		ConstraintRef operator[](int64_t i);
		ConstConstraintRef operator[](int64_t i) const;

		iterator begin() noexcept;
		iterator end() noexcept;
		const_iterator begin() const noexcept;
		const_iterator end() const noexcept;
		
		void shift(int64_t first, int64_t last, int64_t amount);
		void pop(int64_t count);

		int64_t size() const noexcept;
		bool empty() const noexcept;

		void clear();

		// Append another list of constraints to this list, offsetting their particle ids as needed.
		void append(const ConstraintList & other, int32_t offset);

		void append(const ConstraintList& other, int32_t offset, const Transform3& form);
	private:
		struct CVariant {
			Constraint kind;
			int64_t index;
		};

		std::vector<CVariant> constraints;
		std::vector<int32_t> cdata;

	private:
		class iterator {
		public:
			using value_type = ConstraintRef;
			using reference = ConstraintRef;
			using pointer = ConstraintRef;
			using difference_type = ptrdiff_t;
			using iterator_category = std::forward_iterator_tag;

			iterator(const iterator&) = default;
			iterator& operator=(const iterator&) = default;

			iterator()
				: data(nullptr)
			{}
			iterator(int32_t * _data, const std::vector<CVariant>::iterator &_it)
				: data(_data)
				, it(_it)
			{}

			reference operator*() {
				return ConstraintRef(it->kind, data + it->index);
			}
			reference operator->() {
				return ConstraintRef(it->kind, data + it->index);
			}

			iterator& operator++() {
				++it;
				return *this;
			}
			iterator operator++(int) {
				iterator copy = *this;
				++(*this);
				return copy;
			}



			bool operator==(const iterator& other) const noexcept {
				return it == other.it;
			}
			bool operator!=(const iterator& other) const noexcept {
				return it != other.it;
			}
		private:
			friend class const_iterator;
			int32_t * data;
			std::vector<CVariant>::iterator it;
		};

		class const_iterator {
		public:
			using value_type = ConstConstraintRef;
			using reference = ConstConstraintRef;
			using pointer = ConstConstraintRef;
			using difference_type = ptrdiff_t;
			using iterator_category = std::forward_iterator_tag;


			const_iterator(const const_iterator&) = default;
			const_iterator& operator=(const const_iterator&) = default;

			const_iterator()
				: data(nullptr)
			{}
			const_iterator(const int32_t* _data, const std::vector<CVariant>::const_iterator& _it)
				: data(_data)
				, it(_it)
			{}
			const_iterator(const iterator & other)
				: data(other.data)
				, it(other.it)
			{}

			reference operator*() {
				return reference(it->kind, data + it->index);
			}
			reference operator->() {
				return reference(it->kind, data + it->index);
			}

			const_iterator& operator++() {
				++it;
				return *this;
			}
			const_iterator operator++(int) {
				const_iterator copy = *this;
				++(*this);
				return copy;
			}

			bool operator==(const const_iterator& other) const noexcept {
				return it == other.it;
			}
			bool operator!=(const const_iterator& other) const noexcept {
				return it != other.it;
			}
			
		private:
			const int32_t* data;
			std::vector<CVariant>::const_iterator it;
		};
	};
}