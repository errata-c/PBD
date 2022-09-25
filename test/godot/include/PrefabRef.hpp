#pragma once

#include <Godot.hpp>
#include <Reference.hpp>

#include <pbd/prefab/Prefab.hpp>

namespace godot {
	class PrefabRef : public Reference {
		GODOT_CLASS(PrefabRef, Reference)

	private:
		pbd::Prefab prefab;

	public:
		static void _register_methods();

		void _init(); // our initializer called by Godot

		int64_t num_particles() const;
		int64_t num_constraints() const;
		int64_t num_trackers() const;

		int add_particle(Vector3 pos, float mass, float radius);

		void add_distance_constraint(int id0, int id1, float compliance);
		void add_tetra_volume_constraint(int id0, int id1, int id2, int id3, float compliance);
		void add_nh_tetra_volume_constraint(int id0, int id1, int id2, int id3, float hydrostatic_compliance, float deviatoric_compliance);

		void add_tracker(int id0, int id1, int id2, int id3);
	};

}