#include <PrefabRef.hpp>

namespace godot {
	void PrefabRef::_register_methods() {
		register_method("num_particles", &PrefabRef::num_particles);
		register_method("num_constraints", &PrefabRef::num_constraints);
		register_method("num_trackers", &PrefabRef::num_trackers);

		register_method("add_particle", &PrefabRef::add_particle);
		register_method("add_distance_constraint", &PrefabRef::add_distance_constraint);
		register_method("add_tetra_volume_constraint", &PrefabRef::add_tetra_volume_constraint);
		register_method("add_nh_tetra_volume_constraint", &PrefabRef::add_nh_tetra_volume_constraint);

		register_method("add_tracker", &PrefabRef::add_tracker);
	}

	void PrefabRef::_init() {
		// Nothing to do.
	}

	int64_t PrefabRef::num_particles() const {
		return prefab.particles.size();
	}
	int64_t PrefabRef::num_constraints() const {
		return prefab.constraints.size();
	}
	int64_t PrefabRef::num_trackers() const {
		return prefab.trackers.size();
	}

	int PrefabRef::add_particle(Vector3 pos, float mass, float radius) {
		return prefab.add_particle(glm::vec3(pos[0], pos[1], pos[2]), 1.f / mass, radius, 0u);
	}

	void PrefabRef::add_distance_constraint(int id0, int id1, float compliance) {
		glm::vec3
			p0 = prefab.particles[id0].position,
			p1 = prefab.particles[id1].position;

		prefab.add_constraint(pbd::ConstraintDistance{ id0, id1, glm::length(p1 - p0), compliance });
	}
	void PrefabRef::add_tetra_volume_constraint(int id0, int id1, int id2, int id3, float compliance) {
		glm::vec3
			p0 = prefab.particles[id0].position,
			p1 = prefab.particles[id1].position,
			p2 = prefab.particles[id2].position,
			p3 = prefab.particles[id3].position;

		glm::vec3
			t0 = p1 - p0,
			t1 = p2 - p0,
			t3 = p3 - p0;

		glm::vec3 t4 = glm::cross(t0, t1);

		prefab.add_constraint(pbd::ConstraintTetraVolume{
			{id0, id1, id2, id3},
			glm::dot(t3, t4) / 6.f,
			compliance
		});
	}
	void PrefabRef::add_nh_tetra_volume_constraint(int id0, int id1, int id2, int id3, float hydrostatic_compliance, float deviatoric_compliance) {
		glm::vec3
			p0 = prefab.particles[id0].position,
			p1 = prefab.particles[id1].position,
			p2 = prefab.particles[id2].position,
			p3 = prefab.particles[id3].position;

		glm::mat3 restPose;
		restPose[0] = p1 - p0;
		restPose[1] = p2 - p0;
		restPose[2] = p3 - p0;

		restPose = glm::inverse(restPose);

		prefab.add_constraint(pbd::ConstraintNHTetraVolume{
			{id0, id1, id2, id3},
			restPose,
			hydrostatic_compliance,
			deviatoric_compliance
			});
	}

	void PrefabRef::add_tracker(int id0, int id1, int id2, int id3) {
		prefab.add_tracker("", id0, id1, id2, id3);
	}
}