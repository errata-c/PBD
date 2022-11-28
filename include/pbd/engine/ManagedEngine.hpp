#pragma once
#include <pbd/common/Transform.hpp>

#include <vector>
#include <pbd/engine/Engine.hpp>
#include <pbd/engine/ObjectMap.hpp>
#include <pbd/engine/TrackerList.hpp>
#include <pbd/prefab/Prefab.hpp>
#include <pbd/common/Span.hpp>

namespace pbd {
	using particle_span = Span<Particle*>;
	using force_span = Span<vec3_t*>;
	using tracker_span = Span<TransformTracker*>;
	// Cannot implement the constraint span until the constraint list has a random access iterator.

	using const_particle_span = Span<const Particle*>;
	using const_force_span = Span<const vec3_t*>;
	using const_tracker_span = Span<const TransformTracker*>;

	// Engine class that handles the creation and deletion of objects as collections of particles and constraints.
	class ManagedEngine {
	public:
		ManagedEngine(ManagedEngine&&) noexcept = default;
		ManagedEngine& operator=(ManagedEngine&&) noexcept = default;

		ManagedEngine();

		particle_span get_particles(ObjectID id);
		force_span get_forces(ObjectID id);
		tracker_span get_trackers(ObjectID id);

		const_particle_span get_particles(ObjectID id) const;
		const_force_span get_forces(ObjectID id) const;
		const_tracker_span get_trackers(ObjectID id) const;

		ObjectID create(const Prefab& prefab);
		ObjectID create(const Prefab& prefab, const Transform3 & transform);

		bool queue_destroy(ObjectID id);
		size_t num_queued_destroy() const noexcept;

		void destroy_queued();

		size_t num_particles() const noexcept;
		size_t num_constraints() const noexcept;
		size_t num_objects() const noexcept;

		const ParticleList& particles() const;
		const TrackerList & trackers() const;
		const ConstraintList& constraints() const;

		void solve();

		const vec3_t & get_gravity() const noexcept;
		void set_gravity(const vec3_t & _gravity) noexcept;
		void set_substeps(int count);
		int get_substeps() const noexcept;

		void set_static_friction(real_t friction);
		real_t get_static_friction() const noexcept;
		void set_kinetic_friction(real_t friction);
		real_t get_kinetic_friction() const noexcept;

		void set_timestep(real_t dt);
		real_t get_timestep() const noexcept;

		template<typename T>
		int64_t add_constraint(const T& cval) {
			return engine.add_constraint(cval);
		}
	private:
		Engine engine;
		ObjectMap map;
		ObjectQueue queue;
		TrackerList mtrackers;
	};
}