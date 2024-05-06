move_forward () {
	ros2 topic pub /motorControl interfaces/msg/F64vel4 "{ fl: 0.5, fr: 0.5, rl: 0.5, rr: 0.5 }" --rate 20
}

move_backward () {
	ros2 topic pub /motorControl interfaces/msg/F64vel4 "{ fl: -0.5, fr: -0.5, rl: -0.5, rr: -0.5 }" --rate 20
}

move_port () {
	ros2 topic pub /motorControl interfaces/msg/F64vel4 "{ fl: -0.5, fr: 0.5, rl: -0.5, rr: 0.5 }" --rate 20
}

move_starboard () {
	ros2 topic pub /motorControl interfaces/msg/F64vel4 "{ fl: 0.5, fr: -0.5, rl: 0.5, rr: -0.5 }" --rate 20
}
