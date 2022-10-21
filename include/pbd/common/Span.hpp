#pragma once
#include <iterator>
#include <cassert>
#include <type_traits>
#include <stdexcept>
#include <algorithm>

namespace pbd {
	namespace intern {
		template<typename T, typename O = T, typename = int>
		struct eq_exists
			: std::false_type {};

		template<typename T, typename O>
		struct eq_exists<T, O, decltype(std::declval<T>() == std::declval<O>(), 0)>
			: std::true_type {};
		
		template<typename T, typename O = T>
		static constexpr bool eq_exists_v = eq_exists<T, O>::value;

		template<typename T, typename O = T, typename = int>
		struct neq_exists
			: std::false_type {};

		template<typename T, typename O>
		struct neq_exists<T, O, decltype(std::declval<T>() != std::declval<O>(), 0)>
			: std::true_type {};

		template<typename T, typename O = T>
		static constexpr bool neq_exists_v = neq_exists<T, O>::value;
	}

	template<typename Iter>
	class Span {
	public:
		using traits_t = std::iterator_traits<Iter>;
		using tag_t = typename traits_t::iterator_category;

		static_assert(std::is_same_v<tag_t, std::random_access_iterator_tag>, "Must be a random access iterator for pbd::Span to work!");

		using iterator = Iter;
		using const_iterator = iterator;
		using reverse_iterator = std::reverse_iterator<iterator>;

		using value_type = typename traits_t::value_type;
		using pointer = typename traits_t::pointer;
		using reference = typename traits_t::reference;
		using size_type = size_t;
		using difference_type = ptrdiff_t;
		
		static constexpr size_t npos = size_t(-1);
		
		Span() = default;
		~Span() = default;
		Span(const Span &) noexcept = default;
		Span& operator=(const Span&) noexcept = default;

		Span(iterator _first, size_t count) 
			: first(_first)
			, last(_first + count)
		{}
		Span(iterator _first, iterator _last)
			: first(_first)
			, last(_last)
		{}

		template<typename OIter, typename = std::enable_if_t<std::is_nothrow_constructible_v<Iter, const OIter&>>>
		Span(const OIter& _first, const OIter& _last)
			: first(_first)
			, last(_last)
		{}
		template<typename OIter, typename = std::enable_if_t<std::is_nothrow_constructible_v<Iter, const OIter&>>>
		Span(const OIter& _first, size_t count)
			: first(_first)
			, last(_first + count)
		{}

		template<typename OIter, typename = std::enable_if_t<std::is_nothrow_constructible_v<Iter, const OIter&>>>
		Span(const Span<OIter> & other)
			: first(other.begin())
			, last(other.end())
		{}
		template<typename OIter, typename = std::enable_if_t<std::is_nothrow_constructible_v<Iter, const OIter&>>>
		Span& operator=(const Span<OIter>& other) {
			first = other.begin();
			last = other.end();
			return *this;
		}

		reference operator[](size_t i) const noexcept {
			assert(i < size());
			return first[i];
		}
		reference at(size_t i) const {
			if (i >= size()) {
				throw std::out_of_range{"pbd::Span out_of_range exception!"};
			}
			return first[i];
		}

		reference front() const noexcept {
			assert(!empty());
			return *first;
		}
		reference back() const noexcept {
			assert(!empty());
			return *(last -1);
		}

		void swap(Span& other) noexcept {
			std::swap(first, other.first);
			std::swap(last, other.last);
		}
		void trim_front(size_t n) {
			assert(n <= size());
			first += n;
		}
		void trim_back(size_t n) {
			assert(n <= size());
			last -= n;
		}
		Span subspan(size_t pos = 0, size_t count = npos) const {
			if (pos > size()) {
				throw std::out_of_range{ "pbd::Span out_of_range exception!" };
			}
			// Subtraction is necessary here! Using addition with npos (default argument) WILL overflow.
			size_t rcount = std::min(static_cast<size_t>(size() - pos), count);
			iterator start = first + pos;

			return {start, start + rcount};
		}

		size_t size() const noexcept {
			return last - first;
		}
		bool empty() const noexcept {
			return last == first;
		}

		iterator begin() const noexcept {
			return first;
		}
		iterator end() const noexcept {
			return last;
		}

		iterator cbegin() const noexcept {
			return first;
		}
		iterator cend() const noexcept {
			return last;
		}

		reverse_iterator rbegin() const noexcept {
			return std::make_reverse_iterator(last);
		}
		reverse_iterator rend() const noexcept {
			return std::make_reverse_iterator(first);
		}

		reverse_iterator crbegin() const noexcept {
			return std::make_reverse_iterator(last);
		}
		reverse_iterator crend() const noexcept {
			return std::make_reverse_iterator(first);
		}

		/// Operators will only be defined IF the operator is also defined for the value_type.

		template<typename K, typename = std::enable_if_t<(intern::eq_exists_v<value_type, Span<K>::value_type>)>>
		bool operator==(const Span<K>& other) const noexcept {
			if (size() != other.size()) {
				return false;
			}
			else {
				iterator a = begin();
				Span<K>::iterator b = other.begin();
				while (a != last) {
					if (!(*a == *b)) {
						return false;
					}
					++a;
					++b;
				}
				return true;
			}
		}
		template<typename K, typename = std::enable_if_t<(intern::neq_exists_v<value_type, Span<K>::value_type>)>>
		bool operator!=(const Span<K>& other) const noexcept {
			if (size() != other.size()) {
				return false;
			}
			else {
				iterator a = begin();
				Span<K>::iterator b = other.begin();
				while (a != last) {
					if (*a != *b) {
						return false;
					}
					++a;
					++b;
				}
				return true;
			}
		}
	private:
		Iter first, last;
	};
}