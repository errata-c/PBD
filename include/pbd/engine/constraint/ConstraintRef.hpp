#pragma once
#include <cinttypes>
#include <pbd/common/Types.hpp>
#include <pbd/common/Transform.hpp>
#include <pbd/common/Span.hpp>

#include <pbd/engine/constraint/ConstraintType.hpp>

namespace pbd {
	class Engine;
	
	using IdSpan = Span<int32_t*>;
	using ConstIdSpan = Span<const int32_t*>;

	class ConstraintRef {
	public:
		ConstraintRef(const ConstraintRef&) noexcept = default;
		ConstraintRef& operator=(const ConstraintRef&) noexcept = default;

		ConstraintRef(ConstraintType _kind, int32_t* _data) noexcept;

		void eval(Engine& engine, float rdt2) const;

		void remap(int32_t particle_offset, int32_t body_offset);
		void transform(const Transform3& form);

		ConstraintType type() const noexcept;

		IdSpan ids() const noexcept;

		int32_t * data() const noexcept;
	private:
		ConstraintType mkind;
		int32_t* mdata;
	};

	class ConstConstraintRef {
	public:
		ConstConstraintRef(const ConstConstraintRef&) noexcept = default;
		ConstConstraintRef& operator=(const ConstConstraintRef&) noexcept = default;

		ConstConstraintRef(const ConstraintRef & other) noexcept;
		ConstConstraintRef(ConstraintType _kind, int32_t const* _data) noexcept;

		void eval(Engine& engine, float rdt2) const;

		ConstraintType type() const noexcept;

		ConstIdSpan ids() const noexcept;

		const int32_t* data() const noexcept;
	private:
		ConstraintRef ref;
	};
}