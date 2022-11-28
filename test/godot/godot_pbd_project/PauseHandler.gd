extends Node

func _ready():
	get_tree().paused = true

func _input(event):
	if not get_tree().paused:
		return
	
	if event.is_action_pressed("ui_accept"):
		get_tree().paused = false
