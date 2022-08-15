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
		int64_t add(const T& cval) {
			int64_t index = cdata.size();
			Constraint kind = T::Kind;

			const int32_t* ival = (const int32_t*)&cval;
			for (size_t i = 0; i < (sizeof(T) / sizeof(int32_t)); ++i) {
				cdata.push_back(ival[i]);
			}
			constraints.push_back(CVariant{ kind, index });

			return constraints.size() - 1;
		}

		ConstraintRef operator[](int64_t i);
		ConstConstraintRef operator[](int64_t i) const;

		// Erase a single constraint
		void erase(int64_t i);
		// Erase a range of constraints
		void erase(int64_t first, int64_t last);
		

		int64_t size() const noexcept;
		bool empty() const noexcept;

		void clear();

		// Append another list of constraints to this list, offsetting their particle ids as needed.
		void append(const ConstraintList & other, int32_t offset);
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