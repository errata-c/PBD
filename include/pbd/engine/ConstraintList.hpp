#pragma once
#include <vector>
#include <cinttypes>

#include <pbd/collide/Particle.hpp>
#include <pbd/collide/Plane.hpp>

#include <pbd/constraint/Distance.hpp>
#include <pbd/constraint/TetraVolume.hpp>
#include <pbd/constraint/NHTetraVolume.hpp>

#include <pbd/common/ConstraintRef.hpp>

namespace pbd {
	class Engine;

	class ConstraintList {
	public:
		class iterator;
		class const_iterator;

		ConstraintList() = default;
		ConstraintList(const ConstraintList&) = default;
		ConstraintList& operator=(const ConstraintList &) = default;
		ConstraintList(ConstraintList&&) noexcept = default;
		ConstraintList& operator=(ConstraintList&&) noexcept = default;

		template<typename T>
		int64_t addConstraint(const T& cval) {
			int64_t index = cdata.size();
			Constraint kind = T::Kind;

			const int32_t* ival = (const int32_t*)&cval;
			for (size_t i = 0; i < (sizeof(T) / sizeof(int32_t)); ++i) {
				cdata.push_back(ival[i]);
			}
			constraints.push_back(CVariant{ kind, index });

			return constraints.size() - 1;
		}

		
	private:
		struct CVariant {
			Constraint kind;
			int64_t index;
		};

		std::vector<CVariant> constraints;
		std::vector<int32_t> cdata;

	private:
		class iterator {
		public:
			
		};
	};
}