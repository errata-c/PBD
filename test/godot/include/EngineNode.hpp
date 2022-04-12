#include <cinttypes>
#include <Godot.hpp>
#include <Vector3.hpp>
#include <MultiMeshInstance.hpp>

namespace godot {
	class EngineNode : public Node {
		GODOT_CLASS(EngineNode, Node)

	private:
		MultiMeshInstance* mminst;


	public:
		static void _register_methods();

		EngineNode();
		~EngineNode();

		void _init(); // our initializer called by Godot

		void set_multi_mesh_instance(MultiMeshInstance* node);

		int64_t num_particles() const;
		void set_num_particles(int64_t count);

		int64_t create_particles(int64_t count);

		void add_distance_constraint(int id0, int id1, float distance);
		void add_tetra_volume_constraint(int id0, int id1, int id2, int id3, float volume);

		void add_plane_collide(Vector3 origin, Vector3 normal);


	};

}