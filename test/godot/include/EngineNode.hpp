#pragma once
#include <cinttypes>
#include <Godot.hpp>
#include <Vector3.hpp>
#include <MultiMeshInstance.hpp>
#include <PoolArrays.hpp>

#include <Transform.hpp>

#include <PrefabRef.hpp>

#include <pbd/pbd.hpp>
#include <pbd/common/TransformTracker.hpp>
#include <pbd/engine/ManagedEngine.hpp>

namespace godot {
	class PrefabRef;

	class EngineNode : public Node {
		GODOT_CLASS(EngineNode, Node)

	private:
		MultiMeshInstance* mminst;

		PoolRealArray buffer;
		pbd::ManagedEngine engine;
		pbd::TransformTracker tracker;
	public:
		static void _register_methods();

		EngineNode();
		~EngineNode();

		void _init(); // our initializer called by Godot

		PrefabRef* create_empty_prefab();

		void set_force(int32_t id, Vector3 force);
		int32_t nearest_point(Vector3 origin, Vector3 normal) const;
		Vector3 get_position(int32_t id) const;

		int64_t num_particles() const;

		void set_substeps(int count);
		void set_timestep(float delta);

		void set_static_friction(float friction);
		void set_kinetic_friction(float friction);

		uint64_t instance_prefab(PrefabRef * ref);
		void destroy_prefab(uint64_t id);
		void destroy_queued();

		void add_plane_collide(int id, Vector3 origin, Vector3 normal);

		void set_multi_mesh_instance(MultiMeshInstance* node);
		void update_mesh();
		void solve();

		void set_tracker(int id0, int id1, int id2, int id3);
		Transform get_tracker_transform();
	};

}