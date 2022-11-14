#include <pbd/engine/constraint/ConstraintType.hpp>

#include <pbd/engine/constraint/ConstraintMeta.hpp>
#include <pbd/engine/constraint/AllConstraints.hpp>

namespace pbd {
	template<typename T>
	struct SizeOfFunctor {
		size_t operator()() const noexcept {
			return sizeof(T);
		}
	};
	size_t SizeOf(ConstraintType type) noexcept {
		return constraint_visitor<SizeOfFunctor>(type);
	}
}