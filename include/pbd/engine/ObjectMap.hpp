#pragma once
#include <pbd/common/Types.hpp>

#include <vector>
#include <random>
#include <pcg_random.hpp>
#include <parallel_hashmap/phmap.h>

namespace pbd {
	// The range of particles and constraints this object is defined by.
	struct ObjectIndex {
		int32_t pfirst, plast;
		int32_t cfirst, clast;
	};

	class ObjectMap {
	public:
		
	private:
		pcg64 rng;
		phmap::flat_hash_map<ObjectID, ObjectIndex> map;
	};
}