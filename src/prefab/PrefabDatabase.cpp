#include <pbd/prefab/PrefabDatabase.hpp>

#include <fmt/format.h>

#include <ez/serialize.hpp>
#include <ez/deserialize.hpp>

namespace pbd {
	using const_iterator = PrefabDatabase::const_iterator;

	void PrefabDatabase::swap(PrefabDatabase& other) noexcept {
		store.swap(other.store);
	}

	bool PrefabDatabase::isOpen() const noexcept {
		return store.isOpen();
	}

	bool PrefabDatabase::create(const std::filesystem::path& path, bool overwrite) {
		if (store.create(path, overwrite)) {
			if (!store.setKind("pbd")) {
				return false;
			}

			std::string ogtable;
			if (!store.getTable(ogtable)) {
				return false;
			}
			
			if (!store.createTable("prefab_names")) {
				return false;
			}
			if (!store.createTable("prefab_data")) {
				return false;
			}
			if (!store.setDefaultTable("prefab_names")) {
				return false;
			}
			if (!store.setTable("prefab_names")) {
				return false;
			}

			if (!store.eraseTable(ogtable)) {
				return false;
			}

			return true;
		}
		else {
			return false;
		}
	}
	bool PrefabDatabase::open(const std::filesystem::path& path, bool readonly) {
		if (!store.open(path, readonly)) {
			std::string kind;
			if (!store.getKind(kind)) {
				return false;
			}
			if (kind != "pbd") {
				return false;
			}

			// Iterate the elements of the table
			for (auto element : store) {
				prefabNames.push_back(element.key);
			}
			bool res = store.setTable("prefab_data");
			assert(res);



			return true;
		}
		else {
			return false;
		}
	}
	void PrefabDatabase::close() {
		store.close();
		prefabNames.clear();
	}

	size_t PrefabDatabase::numPrefabs() const {
		return prefabNames.size();
	}

	bool PrefabDatabase::load(const_iterator it, Prefab& prefab) const {
		return load(*it, prefab);
	}
	bool PrefabDatabase::load(std::string_view name, Prefab& prefab) const {
		if (!isOpen()) {
			return false;
		}
		if (!contains(name)) {
			return false;
		}

		std::string data;
		bool result = store.get(name, data);
		assert(result);

		uint64_t numParticles = 0, numConstraints = 0, numTrackers = 0;
		
		const char* read = data.data();
		const char* last = read + data.size();

		read = ez::deserialize::u64(read, last, numParticles);
		read = ez::deserialize::u64(read, last, numConstraints);
		read = ez::deserialize::u64(read, last, numTrackers);

		prefab.particles.reserve(numParticles);
		prefab.trackers.reserve(numTrackers);

		// Load all the particles
		for (uint64_t i = 0; i < numParticles; ++i) {
			PrefabParticle particle;

			read = PrefabParticle::deserialize(read, last, particle);

			prefab.particles.push_back(particle);
		}

		// Load all the constraints
		read = ConstraintList::deserialize(read, last, prefab.constraints);

		// Load all the trackers
		for (uint64_t i = 0; i < numParticles; ++i) {
			PrefabTracker tracker;

			read = PrefabTracker::deserialize(read, last, tracker);

			prefab.trackers.push_back(tracker);
		}

		return true;
	}
	bool PrefabDatabase::save(std::string_view name, const Prefab& prefab) {
		if (!isOpen()) {
			return false;
		}

		std::string data;
		uint64_t 
			numParticles = prefab.particles.size(),
			numConstraints = prefab.constraints.size(),
			numTrackers = prefab.trackers.size();

		ez::serialize::u64(numParticles, data);
		ez::serialize::u64(numConstraints, data);
		ez::serialize::u64(numTrackers, data);

		for (uint64_t i = 0; i < numParticles; ++i) {
			PrefabParticle::serialize(prefab.particles[i], data);
		}

		ConstraintList::serialize(prefab.constraints, data);

		for (uint64_t i = 0; i < numTrackers; ++i) {
			PrefabTracker::serialize(prefab.trackers[i], data);
		}

		if (!contains(name)) {
			prefabNames.push_back(std::string(name));
		}

		bool result = store.set(name, data);
		assert(result);

		return true;
	}


	const_iterator PrefabDatabase::find(std::string_view name) const {
		return std::find(prefabNames.begin(), prefabNames.end(), name);
	}

	bool PrefabDatabase::contains(std::string_view name) const {
		return find(name) != prefabNames.end();
	}

	const_iterator PrefabDatabase::begin() const {
		return prefabNames.begin();
	}
	const_iterator PrefabDatabase::end() const {
		return prefabNames.end();
	}

	bool PrefabDatabase::erase(std::string_view name) {
		if (!contains(name)) {
			return false;
		}

		return false;
	}
	const_iterator PrefabDatabase::erase(const_iterator it) {
		assert(it < end());

		return end();
	}
}