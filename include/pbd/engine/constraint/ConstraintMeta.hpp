#pragma once
#include <stdexcept>
#include <pbd/engine/constraint/ConstraintType.hpp>
#include <pbd/engine/constraint/AllConstraints.hpp>

namespace pbd {
	template<template<typename> typename Functor, typename ...Args>
	decltype(auto) constraint_visitor(ConstraintType type, Args&&... args) {
		switch (type) {
		case ConstraintType::Distance:
			return Functor<CDistance>{}(std::forward<Args>(args)...);

		case ConstraintType::Align:
			return Functor<CAlign>{}(std::forward<Args>(args)...);
		case ConstraintType::AttachBody:
			return Functor<CAttachBody>{}(std::forward<Args>(args)...);
		case ConstraintType::AttachParticle:
			return Functor<CAttachParticle>{}(std::forward<Args>(args)...);
		case ConstraintType::HingeJoint:
			return Functor<CHingeJoint>{}(std::forward<Args>(args)...);
		case ConstraintType::PrismaticJoint:
			return Functor<CPrismaticJoint>{}(std::forward<Args>(args)...);
		case ConstraintType::SphereJoint:
			return Functor<CSphereJoint>{}(std::forward<Args>(args)...);

		case ConstraintType::Tetra:
			return Functor<CTetra>{}(std::forward<Args>(args)...);
		case ConstraintType::NHTetra:
			return Functor<CNHTetra>{}(std::forward<Args>(args)...);
		default:
			throw std::logic_error("Unimplemented!");
		}
	}
}