extends Spatial

var engine

var p = [
		Vector3(3,3,0),
		Vector3(-1.5,3,2.598),
		Vector3(-1.5,3,-2.598),
		Vector3(0,5,0),
		Vector3(0,1,0),
	]
	
var drag = -1
var drag_offset: Vector3
var drag_interp: float
var drag_force: float = 3.0
var drag_prior: Vector2

# Called when the node enters the scene tree for the first time.
func _ready():
	engine = $Engine
	engine.set_multi_mesh_instance($Particles)
	engine.set_substeps(8)
	engine.set_timestep(1.0 / 60.0)
	
	engine.set_static_friction(0.1)
	engine.set_kinetic_friction(0.05)
	
	var radius = 0.9
	var imass = 1.0 / 0.3
	
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
	engine.add_nh_tetra_volume_constraint(0,1,2,4, compliance)
	
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
		for id in range(len(p)):
			engine.add_plane_collide(id, origins[i], normals[i])
	
	engine.update_mesh()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(delta):
	if drag != -1:
		var camera = $Camera
		var origin = camera.project_ray_origin(drag_prior)
		var normal = camera.project_ray_normal(drag_prior)
		
		var pos = engine.get_position(drag) + drag_offset
		var target = origin + normal * drag_interp
		var force = target - pos
		
		engine.set_force(drag, force * drag_force)
	
	engine.solve()
	engine.update_mesh()
	

func _input(event):
	if event is InputEventMouseButton and event.button_index == 1:
		if event.pressed:
			var camera = $Camera
			var origin = camera.project_ray_origin(event.position)
			var normal = camera.project_ray_normal(event.position)
			
			var id = engine.nearest_point(origin, normal)
			drag = id
			
			var pos = engine.get_position(id)
			drag_interp = normal.dot(pos - origin)
			drag_offset = pos - (origin + normal * drag_interp)
			drag_prior = event.position
			
		elif drag != -1:
			drag = -1
	elif drag != -1 and event is InputEventMouseMotion:
		drag_prior = event.position
