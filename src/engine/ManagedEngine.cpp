#include <pbd/engine/ManagedEngine.hpp>

namespace pbd {
	ManagedEngine::ManagedEngine()
	{}

	ObjectID ManagedEngine::create(const Prefab& prefab) {
		return create(prefab, Transform3());
	}
	ObjectID ManagedEngine::create(const Prefab& prefab, const Transform3 & form) {
		// Instance the prefab.
		// First reserve the range of particles, constraints, and trackers for it.
		// Get the offset for each
		// Get a new ObjectID from the map.
		// Create a transform matrix for the particles, make sure to update the constraints as well.
		
		int32_t pfirst = static_cast<int32_t>(engine.num_particles());
		int32_t cfirst = static_cast<int32_t>(engine.num_constraints());
		int32_t tfirst = static_cast<int32_t>(trackers.size());

		int32_t plast = pfirst + static_cast<int32_t>(prefab.num_particles());
		int32_t clast = cfirst + static_cast<int32_t>(prefab.num_constraints());
		int32_t tlast = tfirst + static_cast<int32_t>(prefab.num_trackers());

		ObjectID id = map.create(IndexRange(pfirst, plast), IndexRange(cfirst, clast), IndexRange(tfirst, tlast));

		for (const PrefabParticle & part: prefab.particles) {
			engine.particle.add(part, form);
		}

		// Trackers reference particles, so offset by the first particle index.
		for (const PrefabTracker & tracker: prefab.trackers) {
			trackers.add(tracker, engine, pfirst);
		}

		// Constraints reference particles, so offset by the first particle index.
		engine.constraints.append(prefab.constraints, pfirst, form);

		// Fin
		return id;
	}

	bool ManagedEngine::queue_destroy(ObjectID id) {
		return queue.push(id);
	}
	size_t ManagedEngine::num_queued_destroy() const noexcept {
		return queue.size();
	}

	void ManagedEngine::destroy_queued() {
		// The locations to write to.
		int32_t wp = 0, wc = 0, wt = 0;

		for (const ObjectInfo & obj : map) {
			// If the object is in the queue, then skip it.
			if (queue.contains(obj.id())) {
				continue;
			}
			else {
				// Shift everything down.
				engine.particle.shift(obj.particles().first, obj.particles().last, wp - obj.particles().first);
				engine.constraints.shift(obj.constraints().first, obj.constraints().last, wc - obj.constraints().first);
				trackers.shift(obj.trackers().first, obj.trackers().last, wt - obj.trackers().first);

				// Update the write positions
				wp += obj.particles().size();
				wc += obj.constraints().size();
				wt += obj.trackers().size();
			}
		}

		// Destroy the entries in the map.
		map.destroy(queue);
		
		// Finish up by erasing the unused back elements.
		if (engine.particle.size() > wp) {
			engine.particle.pop(engine.particle.size() - wp);
		}
		if (engine.constraints.size() > wc) {
			engine.constraints.pop(engine.constraints.size() - wc);
		}
		if (trackers.size() > wt) {
			trackers.pop(trackers.size() - wt);
		}
		
	}

	size_t ManagedEngine::num_particles() const noexcept {
		return engine.num_particles();
	}
	size_t ManagedEngine::num_constraints() const noexcept {
		return engine.num_constraints();
	}
	size_t ManagedEngine::num_objects() const noexcept {
		return map.size();
	}

	void ManagedEngine::solve() {
		engine.solve();
	}

	const glm::vec3& ManagedEngine::get_gravity() const noexcept {
		return engine.gravity;
	}
	void ManagedEngine::set_gravity(const glm::vec3& _gravity) noexcept {
		engine.gravity = _gravity;
	}
	void ManagedEngine::set_substeps(int count) {
		assert(count >= 1);
		engine.substeps = count;
	}
	int ManagedEngine::get_substeps() const noexcept {
		return engine.substeps;
	}
}