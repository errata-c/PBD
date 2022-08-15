#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>

namespace pbd {
	class Engine;

	class ConstraintRef {
	public:
		ConstraintRef(const ConstraintRef&) noexcept = default;
		ConstraintRef& operator=(const ConstraintRef&) noexcept = default;

		ConstraintRef(Constraint _kind, int32_t* _data) noexcept;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t offset);

		Constraint type() const noexcept;

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

		const int32_t* data() const noexcept;
	private:
		ConstraintRef ref;
	};
}