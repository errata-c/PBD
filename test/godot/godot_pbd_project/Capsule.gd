tool
extends Spatial

var _height = 1.0
var _radius = 1.0

export var height: float = 1.0 setget set_height, get_height
export var radius: float = 1.0 setget set_radius, get_radius

func set_height(val):
	_height = val
	$cy.scale.y = val
	$cap0.translation.z = val - 0.002
	$cap1.translation.z = -val + 0.002

func get_height():
	return _height

func set_radius(val):
	_radius = val
	$cy.scale.x = val - 0.002
	$cy.scale.z = val - 0.002
	$cap0.scale = Vector3(val, val, val)
	$cap1.scale = -Vector3(val, val, val)

func get_radius():
	return _radius
