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
	private:
		Constraint kind;
		int32_t* data;
	};
}