#pragma once
#include <iterator>
#include <cinttypes>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>



namespace pbd {
	class Engine;

	glm::vec3 rotate(const glm::quat& rot, const glm::vec3& vec);
	glm::vec3 reverse_rotate(const glm::quat& rot, const glm::vec3& vec);

	glm::vec3 perpendicular(const glm::vec3 & x, const glm::vec3 & n);

	glm::vec3 friction_delta(const glm::vec3 & perp, float overlap, float sfriction, float kfriction);

	float tetrahedron_volume(const glm::vec3 & p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3);

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