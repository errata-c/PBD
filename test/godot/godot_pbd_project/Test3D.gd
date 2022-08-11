extends Spatial

var engine

# Called when the node enters the scene tree for the first time.
func _ready():
	engine = $Engine
	engine.set_multi_mesh_instance($Particles)
	engine.set_substeps(20)
	engine.set_timestep(1.0 / 60.0)
	
	engine.set_static_friction(0.9)
	engine.set_kinetic_friction(0.6)
	
	var radius = 0.9
	var imass = 1.0 / 0.5
	
	var p = [
		Vector3(3,3,0),
		Vector3(-1.5,3,2.598),
		Vector3(-1.5,3,-2.598),
		Vector3(0,5,0),
	]
	
	for x in p:
		engine.add_particle(
			x,
			imass,
			radius)
	
	var compliance = 0.005
	
	#engine.add_distance_constraint(0,1, compliance)
	#engine.add_distance_constraint(0,2, compliance)
	#engine.add_distance_constraint(0,3, compliance)
	#engine.add_distance_constraint(1,2, compliance)
	#engine.add_distance_constraint(1,3, compliance)
	#engine.add_distance_constraint(2,3, compliance)
	
	#engine.add_tetra_volume_constraint(0,1,2,3, compliance)
	
	# 0,1,2,3  1,0,2,3  1,2,3,0
	
	engine.add_nh_tetra_volume_constraint(0,1,2,3, compliance)
	
	var origins = [
		Vector3(0,0,0),
		#Vector3(10,0,0),
		#Vector3(-10,0,0),
		#Vector3(0,0,10),
		#Vector3(0,0,-10)
	]
	var normals = [
		Vector3(0,1,0),
		#Vector3(-1,0,0),
		#Vector3(1,0,0),
		#Vector3(0,0,-1),
		#Vector3(0,0,1)
	]
	
	for i in range(len(origins)):
		for id in range(4):
			engine.add_plane_collide(id, origins[i], normals[i])
	
	engine.update_mesh()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	engine.solve()
	engine.update_mesh()
