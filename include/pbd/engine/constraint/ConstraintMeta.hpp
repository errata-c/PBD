#pragma once
#include <stdexcept>
#include <pbd/engine/constraint/AllConstraints.hpp>

namespace pbd {
	template<template<typename> typename Functor, typename ...Args>
	decltype(auto) constraint_visitor(Constraint type, Args&&... args) {
		switch (type) {
		case Constraint::Distance:
			return Functor<CDistance>{}(std::forward<Args>(args)...);

		case Constraint::Align:
			return Functor<CAlign>{}(std::forward<Args>(args)...);
		case Constraint::AttachBody:
			throw std::logic_error("Unimplemented!");
			//return Functor<CollidePlane>{}(std::forward<Args>(args)...);
		case Constraint::AttachParticle:
			throw std::logic_error("Unimplemented!");
			//return Functor<CollidePlane>{}(std::forward<Args>(args)...);
		case Constraint::HingeJoint:
			return Functor<CHingeJoint>{}(std::forward<Args>(args)...);
		case Constraint::PrismaticJoint:
			return Functor<CPrismaticJoint>{}(std::forward<Args>(args)...);
		case Constraint::SphereJoint:
			return Functor<CSphereJoint>{}(std::forward<Args>(args)...);

		case Constraint::Tetra:
			return Functor<CTetra>{}(std::forward<Args>(args)...);
		case Constraint::NHTetra:
			return Functor<CNHTetra>{}(std::forward<Args>(args)...);
		default:
			throw std::logic_error("Unimplemented!");
		}
	}
}