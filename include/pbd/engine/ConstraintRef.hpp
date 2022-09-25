#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>

namespace pbd {
	class Engine;
	struct ConstraintDataRange {
		int32_t *first, *last;

		int32_t* begin() const noexcept {
			return first;
		}
		int32_t* end() const noexcept {
			return last;
		}
	};
	struct ConstConstraintDataRange {
		const int32_t* first, * last;

		const int32_t* begin() const noexcept {
			return first;
		}
		const int32_t* end() const noexcept {
			return last;
		}
	};

	class ConstraintRef {
	public:
		ConstraintRef(const ConstraintRef&) noexcept = default;
		ConstraintRef& operator=(const ConstraintRef&) noexcept = default;

		ConstraintRef(Constraint _kind, int32_t* _data) noexcept;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);
		void transform(const Transform3& form);

		Constraint type() const noexcept;

		ConstraintDataRange ids() const noexcept;

		int32_t * data() const noexcept;
	private:
		Constraint mkind;
		int32_t* mdata;
	};

	class ConstConstraintRef {
	public:
		ConstConstraintRef(const ConstConstraintRef&) noexcept = default;
		ConstConstraintRef& operator=(const ConstConstraintRef&) noexcept = default;

		ConstConstraintRef(const ConstraintRef & other) noexcept;
		ConstConstraintRef(Constraint _kind, int32_t const* _data) noexcept;

		void eval(Engine& engine, float rdt2) const;

		Constraint type() const noexcept;

		ConstConstraintDataRange ids() const noexcept;

		const int32_t* data() const noexcept;
	private:
		ConstraintRef ref;
	};
}