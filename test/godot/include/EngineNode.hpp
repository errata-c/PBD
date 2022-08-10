#include <cinttypes>
#include <Godot.hpp>
#include <Vector3.hpp>
#include <MultiMeshInstance.hpp>
#include <PoolArrays.hpp>

#include <pbd/Engine.hpp>

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

		int64_t num_particles() const;
		void set_num_particles(int64_t count);

		void set_particle(int index, Vector3 pos, float mass);

		void add_distance_constraint(int id0, int id1);
		void add_tetra_volume_constraint(int id0, int id1, int id2, int id3);

		void add_plane_collide(Vector3 origin, Vector3 normal);

		void set_multi_mesh_instance(MultiMeshInstance* node);
		void update_mesh();
		void solve();
	};

}