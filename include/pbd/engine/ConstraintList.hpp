#pragma once
#include <vector>
#include <string>
#include <cinttypes>

#include <pbd/common/Utils.hpp>
#include <pbd/common/Transform.hpp>

#include <pbd/engine/collide/Particle.hpp>
#include <pbd/engine/collide/Plane.hpp>

#include <pbd/engine/constraint/Distance.hpp>
#include <pbd/engine/constraint/TetraVolume.hpp>
#include <pbd/engine/constraint/NHTetraVolume.hpp>

#include <pbd/engine/ConstraintRef.hpp>

namespace pbd {
	class Engine;

	class ConstraintList {
		struct CVariant {
			Constraint kind;
			int64_t index;
		};
		class data_iterator : public std::vector<CVariant>::iterator {
		public:
			using parent_t = std::vector<CVariant>::iterator;
			int32_t* data;

			data_iterator()
				: data(nullptr)
			{}
			data_iterator(int32_t* _data, const parent_t& _it)
				: parent_t(_it)
				, data(_data)
			{}
		};
		class const_data_iterator : public std::vector<CVariant>::const_iterator {
		public:
			using parent_t = std::vector<CVariant>::const_iterator;
			const int32_t* data;

			const_data_iterator()
				: data(nullptr)
			{}
			const_data_iterator(const int32_t* _data, const parent_t& _it)
				: parent_t(_it)
				, data(_data)
			{}
			const_data_iterator(const data_iterator& _it)
				: parent_t(_it)
				, data(_it.data)
			{}
		};

		struct IterAdapt {
			static ConstraintRef adapt(const data_iterator& it) {
				return ConstraintRef(it->kind, it.data + it->index);
			}
			static ConstConstraintRef adapt(const const_data_iterator& it) {
				return ConstConstraintRef(it->kind, it.data + it->index);
			}
		};
	public:
		using iterator = iterator_adaptor<data_iterator, IterAdapt>;
		using const_iterator = iterator_adaptor<const_data_iterator, IterAdapt>;

		static void serialize(const ConstraintList & clist, std::string& output);
		static const char* deserialize(const char * first, const char * last, ConstraintList & clist);

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

		iterator begin() noexcept;
		iterator end() noexcept;
		const_iterator begin() const noexcept;
		const_iterator end() const noexcept;
		
		void shift(int64_t first, int64_t last, int64_t amount);
		void pop(int64_t count);

		int64_t size() const noexcept;
		bool empty() const noexcept;

		void clear();

		// Append another list of constraints to this list, offsetting their particle ids as needed.
		void append(const ConstraintList & other, int32_t offset);

		void append(const ConstraintList& other, int32_t offset, const Transform3& form);
	private:
		std::vector<CVariant> constraints;
		std::vector<int32_t> cdata;
	};
}