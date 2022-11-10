#include <pbd/engine/constraint/ConstraintRef.hpp>
#include <pbd/engine/constraint/AllConstraints.hpp>

namespace pbd {
	ConstraintRef::ConstraintRef(Constraint _kind, int32_t* _data) noexcept
		: mkind(_kind)
		, mdata(_data)
	{}

	void ConstraintRef::eval(Engine& engine, float rdt2) const {
		switch (type()) {
		case Constraint::Distance:
			((CDistance*)mdata)->eval(engine, rdt2);
			break;
		case Constraint::Tetra:
			((CTetra*)mdata)->eval(engine, rdt2);
			break;
		case Constraint::NHTetra:
			((CNHTetra*)mdata)->eval(engine, rdt2);
			break;
		case Constraint::CollidePlane:
			//((CollidePlane*)mdata)->eval(engine, rdt2);
			break;
		}
	}

	void ConstraintRef::remap(int32_t offset) {
		for (int32_t & id: ids()) {
			id += offset;
		}
	}
	void ConstraintRef::transform(const Transform3& form) {
		switch (type()) {
		case Constraint::Distance:
			((CDistance*)mdata)->transform(form);
			break;
		case Constraint::Tetra:
			((CTetra*)mdata)->transform(form);
			break;
		case Constraint::NHTetra:
			((CNHTetra*)mdata)->transform(form);
			break;
		}
	}

	Constraint ConstraintRef::type() const noexcept {
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
	ConstConstraintRef::ConstConstraintRef(Constraint _kind, int32_t const* _data) noexcept
		: ref(_kind, const_cast<int32_t*>(_data))
	{}

	void ConstConstraintRef::eval(Engine& engine, float rdt2) const {
		return ref.eval(engine, rdt2);
	}

	Constraint ConstConstraintRef::type() const noexcept {
		return ref.type();
	}

	ConstIdSpan ConstConstraintRef::ids() const noexcept {
		return ConstIdSpan{ data(), NumIds(type()) };
	}

	const int32_t* ConstConstraintRef::data() const noexcept {
		return ref.data();
	}
}