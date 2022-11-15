#include <pbd/engine/constraint/ConstraintRef.hpp>
#include <pbd/engine/constraint/AllConstraints.hpp>

#include <pbd/engine/constraint/ConstraintMeta.hpp>

namespace pbd {
	ConstraintRef::ConstraintRef(ConstraintType _kind, int32_t* _data) noexcept
		: mkind(_kind)
		, mdata(_data)
	{}

	template<typename T>
	struct CEvalFunctor {
		void operator()(void * mdata, Engine& engine, float rdt2) {
			((T*)mdata)->eval(engine, rdt2);
		}
	};
	void ConstraintRef::eval(Engine& engine, float rdt2) const {
		constraint_visitor<CEvalFunctor>(mkind, mdata, engine, rdt2);
	}

	template<typename T>
	struct CRemapFunctor {
		void operator()(void* mdata, int32_t particle_offset, int32_t body_offset) {
			((T*)mdata)->remap(particle_offset, body_offset);
		}
	};
	void ConstraintRef::remap(int32_t particle_offset, int32_t body_offset) {
		constraint_visitor<CRemapFunctor>(mkind, mdata, particle_offset, body_offset);
	}

	template<typename T>
	struct CTransformFunctor {
		void operator()(void* mdata, const Transform3& form) {
			((T*)mdata)->transform(form);
		}
	};
	void ConstraintRef::transform(const Transform3& form) {
		constraint_visitor<CTransformFunctor>(mkind, mdata, form);
	}

	ConstraintType ConstraintRef::type() const noexcept {
		return mkind;
	}

	IdSpan ConstraintRef::ids() const noexcept {
		return IdSpan{data(), NumIds(type())};
	}

	int32_t* ConstraintRef::data() const noexcept {
		return mdata;
	}



	ConstConstraintRef::ConstConstraintRef(const ConstraintRef& other) noexcept
		: ref(other)
	{}
	ConstConstraintRef::ConstConstraintRef(ConstraintType _kind, int32_t const* _data) noexcept
		: ref(_kind, const_cast<int32_t*>(_data))
	{}

	void ConstConstraintRef::eval(Engine& engine, float rdt2) const {
		return ref.eval(engine, rdt2);
	}

	ConstraintType ConstConstraintRef::type() const noexcept {
		return ref.type();
	}

	ConstIdSpan ConstConstraintRef::ids() const noexcept {
		return ConstIdSpan{ data(), NumIds(type()) };
	}

	const int32_t* ConstConstraintRef::data() const noexcept {
		return ref.data();
	}
}