#include <cinttypes>
#include <Godot.hpp>
#include <Vector3.hpp>
#include <MultiMeshInstance.hpp>
#include <PoolArrays.hpp>

#include <pbd/pbd.hpp>

namespace godot {
	class EngineNode : public Node {
		GODOT_CLASS(EngineNode, Node)

	private:
		MultiMeshInstance* mminst;

		PoolRealArray buffer;
		pbd::Engine engine;
	public:
		static void _register_methods();

		EngineNode();
		~EngineNode();

		void _init(); // our initializer called by Godot

		void set_force(int32_t id, Vector3 force);

		int32_t nearest_point(Vector3 origin, Vector3 normal) const;

		Vector3 get_position(int32_t id) const;

		int64_t num_particles() const;

		void set_substeps(int count);
		void set_timestep(float delta);

		void set_static_friction(float friction);
		void set_kinetic_friction(float friction);

		void add_particle(Vector3 pos, float mass, float radius);

		void add_distance_constraint(int id0, int id1, float compliance);
		void add_tetra_volume_constraint(int id0, int id1, int id2, int id3, float compliance);
		void add_nh_tetra_volume_constraint(int id0, int id1, int id2, int id3, float compliance);

		void add_plane_collide(int id, Vector3 origin, Vector3 normal);

		void set_multi_mesh_instance(MultiMeshInstance* node);
		void update_mesh();
		void solve();
	};

}