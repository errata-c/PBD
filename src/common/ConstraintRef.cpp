#include <pbd/common/ConstraintRef.hpp>

#include <pbd/collide/Particle.hpp>
#include <pbd/collide/Plane.hpp>

#include <pbd/constraint/Distance.hpp>
#include <pbd/constraint/TetraVolume.hpp>
#include <pbd/constraint/NHTetraVolume.hpp>

namespace pbd {
	ConstraintRef::ConstraintRef(Constraint _kind, int32_t* _data) noexcept
		: kind(_kind)
		, data(_data)
	{}

	void ConstraintRef::eval(Engine& engine, float rdt2) const {
		switch (type()) {
		case Constraint::Distance:
			((ConstraintDistance*)data)->eval(engine, rdt2);
			break;
		case Constraint::TetraVolume:
			((ConstraintTetraVolume*)data)->eval(engine, rdt2);
			break;
		case Constraint::NHTetraVolume:
			((ConstraintNHTetraVolume*)data)->eval(engine, rdt2);
			break;
		case Constraint::CollideParticle:
			((CollideParticle*)data)->eval(engine, rdt2);
			break;
		case Constraint::CollidePlane:
			((CollidePlane*)data)->eval(engine, rdt2);
			break;
		}	
	}

	void ConstraintRef::remap(int32_t offset) {
		switch (type()) {
		case Constraint::Distance:
			data[0] += offset;
			data[1] += offset;
			break;
		case Constraint::TetraVolume:
			data[0] += offset;
			data[1] += offset;
			data[2] += offset;
			data[3] += offset;
			break;
		case Constraint::NHTetraVolume:
			data[0] += offset;
			data[1] += offset;
			data[2] += offset;
			data[3] += offset;
			break;
		case Constraint::CollideParticle:
			data[0] += offset;
			data[1] += offset;
			break;
		case Constraint::CollidePlane:
			data[0] += offset;
			break;
		}
	}

	Constraint ConstraintRef::type() const noexcept {
		return kind;
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
}