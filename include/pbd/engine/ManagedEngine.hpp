#pragma once
#include <vector>
#include <pbd/engine/Engine.hpp>
#include <pbd/engine/ObjectMap.hpp>
#include <pbd/engine/TrackerList.hpp>
#include <pbd/prefab/Prefab.hpp>

namespace pbd {
	// Engine class that handles the creation and deletion of objects as collections of particles and constraints.
	class ManagedEngine {
	public:
		ManagedEngine(ManagedEngine&&) noexcept = default;
		ManagedEngine& operator=(ManagedEngine&&) noexcept = default;

		ManagedEngine();

		ObjectID create(const Prefab & prefab);
		bool queue_destroy(ObjectID id);
		size_t num_queued_destroy() const noexcept;

		void destroy_queued();

		size_t num_particles() const noexcept;
		size_t num_constraints() const noexcept;
		size_t num_objects() const noexcept;

		void solve();

		const glm::vec3 & get_gravity() const noexcept;
		void set_gravity(const glm::vec3 & _gravity) noexcept;
		void set_substeps(int count);
		int get_substeps() const noexcept;

	private:
		Engine engine;
		ObjectMap map;
		ObjectQueue queue;
		TrackerList trackers;
	};
}