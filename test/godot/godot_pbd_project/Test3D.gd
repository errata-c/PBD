extends Spatial

var engine

var p = [
		Vector3(3,3,0),
		Vector3(-1.5,3,2.598),
		Vector3(-1.5,3,-2.598),
		
		Vector3(0,5,0), # Top
		Vector3(0,1,0), # Bot
	]
	
var drag = -1
var drag_offset: Vector3
var drag_interp: float
var drag_force: float = 3.0
var drag_prior: Vector2

var skeleton

var bodies = []

# Called when the node enters the scene tree for the first time.
func _ready():
	skeleton = $monkey/Armature/Skeleton
	
	engine = $Engine
	engine.set_multi_mesh_instance($Particles)
	engine.set_substeps(12)
	engine.set_timestep(1.0 / 60.0)
	
	engine.set_static_friction(0.1)
	engine.set_kinetic_friction(0.2)
	
	var tets = [
		[0,1,2,3],
		[0,1,2,4]
	]
	
	var tot_mass = 0.0
	for tet in tets:
		tot_mass += 0.1 * tet_volume(p[tet[0]],p[tet[1]],p[tet[2]],p[tet[3]])
	
	var radius = 0.9
	var imass = 0.0
	
	for x in p:
		engine.add_particle(
			x,
			imass,
			radius)
		
		imass = len(p) / tot_mass
	
	var compliance = 1e-3
	
	# 0,1,2,3  1,0,2,3  1,2,3,0
	
	for tet in tets:
		engine.add_nh_tetra_volume_constraint(
			tet[0], 
			tet[1], 
			tet[2], 
			tet[3], 
			1e-3, # Hydrostatic (volume)
			1e-3) # Deviatoric (shear)
	
	engine.update_mesh()
	
	# Ordering of indices doesn't matter anymore, except to determine the origin
	engine.set_tracker(3, 0, 1, 2)
	
	
	bodies.append(engine.add_capsule(
		$Capsule.transform.origin, 
		$Capsule.transform.basis,
		0.0,
		$Capsule.height,
		$Capsule.radius 
	))
	bodies.append(engine.add_capsule(
		$Capsule2.transform.origin, 
		$Capsule2.transform.basis,
		1.0,
		$Capsule2.height,
		$Capsule2.radius 
	))
	
	var joint = $Capsule.translation
	joint.y = 0.0
	engine.add_hinge_joint(bodies[0], bodies[1], joint, Vector3(1,0,0))

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
	
	var form = engine.get_tracker_transform()
	skeleton.set_bone_pose(0, form)
	
	$Capsule.transform = engine.get_rigid_body_transform(bodies[0])
	$Capsule2.transform = engine.get_rigid_body_transform(bodies[1])

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

func tet_volume(p0, p1, p2, p3):
	var d0 = p1 - p0
	var d1 = p2 - p0
	var d2 = p3 - p0
	var basis = Basis(d0, d1, d2)
	return abs(basis.determinant()) / 6.0
