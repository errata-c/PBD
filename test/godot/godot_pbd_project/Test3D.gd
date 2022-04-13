extends Spatial

var engine

# Called when the node enters the scene tree for the first time.
func _ready():
	engine = $Engine
	engine.set_multi_mesh_instance($Particles)
	engine.set_num_particles(4)
	
	engine.set_particle(0,
		Vector3(3,3,0),
		1.0)
	engine.set_particle(1,
		Vector3(-1.5,3,2.598),
		1.0)
	engine.set_particle(2,
		Vector3(-1.5,3,-2.598),
		1.0)
	engine.set_particle(3,
		Vector3(0,5,0),
		1.0)
		
	engine.add_distance_constraint(0,1)
	engine.add_distance_constraint(0,2)
	engine.add_distance_constraint(0,3)
	engine.add_distance_constraint(1,2)
	engine.add_distance_constraint(1,3)
	engine.add_distance_constraint(2,3)
	engine.add_tetra_volume_constraint(0,1,2,3)
	
	#engine.add_plane_collide(
	#	Vector3(0,0,0),
	#	Vector3(0,1,0)
	#)
	
	#engine.add_plane_collide(
	#	Vector3(10,0,0),
	#	Vector3(-1,0,0)
	#)
	#engine.add_plane_collide(
	#	Vector3(-10,0,0),
	#	Vector3(1,0,0)
	#)
	#engine.add_plane_collide(
	#	Vector3(0,0,10),
	#	Vector3(0,0,-1)
	#)
	#engine.add_plane_collide(
	#	Vector3(0,0,-10),
	#	Vector3(0,0,1)
	#)
	
	engine.update_mesh()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	engine.solve()
	engine.update_mesh()
