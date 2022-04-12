#include <EngineNode.hpp>


namespace godot {
	void EngineNode::_register_methods() {

	}

	EngineNode::EngineNode() 
		: mminst(nullptr)
	{

	}
	EngineNode::~EngineNode() {

	}

	void EngineNode::_init() {

	}

	void EngineNode::set_multi_mesh_instance(MultiMeshInstance* node) {

	}

	int64_t EngineNode::num_particles() const {
		return 0;
	}
	void EngineNode::set_num_particles(int64_t count) {

	}

	int64_t EngineNode::create_particles(int64_t count) {
		return 0;
	}

	void EngineNode::add_distance_constraint(int id0, int id1, float distance) {

	}
	void EngineNode::add_tetra_volume_constraint(int id0, int id1, int id2, int id3, float volume) {

	}

	void EngineNode::add_plane_collide(Vector3 origin, Vector3 normal) {

	}
}