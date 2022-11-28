#pragma once
#include <iterator>
#include <cinttypes>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;

	vec3_t rotate(const quat_t& rot, const vec3_t& vec);
	vec3_t reverse_rotate(const quat_t& rot, const vec3_t& vec);

	vec3_t perpendicular(const vec3_t & x, const vec3_t & n);

	vec3_t friction_delta(const vec3_t & perp, real_t overlap, real_t sfriction, real_t kfriction);

	real_t tetrahedron_volume(const vec3_t & p0, const vec3_t& p1, const vec3_t& p2, const vec3_t& p3);

	// By default, assume the SFunction type has a static function called 'adapt', accepting a pointer or reference to value_type.
	template<typename Iter, typename SFunction>
	class iterator_adaptor : public Iter {
	public:
		using parent_t = Iter;
		using traits_t = std::iterator_traits<Iter>;

		using parent_value_type = typename traits_t::value_type;
		using parent_pointer = typename traits_t::pointer;
		using parent_reference = typename traits_t::reference;
		using iterator_category = typename traits_t::iterator_category;
		using parent_deref_type = decltype(std::declval<parent_t>().operator*());

		static_assert(std::is_nothrow_copy_constructible_v<parent_t>, "iterator_adaptor requires a nothrow copy constructible iterator type!");

		using rtype = decltype(SFunction::adapt(std::declval<parent_t>()));

		using Iter::Iter;

		// if its a reference type then remove reference
		// if its a pointer, treat as value_type

		using value_type = std::remove_reference_t<rtype>;
		using pointer = std::conditional_t<std::is_lvalue_reference_v<rtype>, value_type*, value_type>;
		using reference = rtype;
		using difference_type = typename std::ptrdiff_t;

		reference operator*() {
			return SFunction::apply(*this);
		}
		pointer operator->() {
			if constexpr (std::is_lvalue_reference_v<rtype>) {
				return std::addressof(operator*());
			}
			else {
				return operator*();
			}
		}
	};
}