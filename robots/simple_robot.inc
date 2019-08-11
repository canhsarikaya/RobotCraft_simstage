define laser ranger(
	sensor
	(
		range [0.0 5.6]
		fov 240.0
		samples 240
	)

	color "dark gray"
	size [0.06 0.06 0.2]

	block
	(
		points 3
		point[0] [0 0]
		point[1] [1 0.5]
		point[2] [0 1]
		z [0 0.2]
	)
)

define rhomboid actuator(
	color "DarkSeaGreen4"
	size [0.13 0.15 0.08]

	block
    (
        points 4
        point[0] [0.125 0.25]
        point[1] [0.625 0.5]
        point[2] [0.625 0.75]
		point[3] [0.125 1]
	    z [0.0 0.08]			#its z coordinate starts from simple_robot position
    )	  
)

define screen actuator(
	color "royal blue"
	size [0.06 0.05 0.08]

	block
    (
        points 4
        point[0] [0.625 0.5]
        point[1] [0.1 0.5]
        point[2] [0.1 0.75]
		point[3] [0.625 0.75]
	    z [0.0 0.08]			
    )	  
)

define junction_top actuator(
	color "dark gray"
	size [0.015 0.015 0.03]

	block
    (
        points 4
        point[0] [0.25 0.5]
        point[1] [0.5 0.5]
        point[2] [0.5 0.75]
        point[3] [0.25 0.75]
	    z [0.08 0.11]
    )
)

define junction_low actuator(
	color "dark gray"
	size [0.015 0.03 0.015]

	block
    (
        points 4
        point[0] [0.25 0.5]
        point[1] [0.5 0.5]
		point[2] [0.5 0.75]
		point[3] [0.25 0.75]
		z [0.08 0.11]
	)
)

define propeller1 actuator(
	color "black"
	size [0.6 0.03 0.001]	

	block
	(
		points 4
		point[0] [0.3125 0.5625]
		point[1] [0.75 0.5625]
		point[2] [0.75 0.6875]
		point[3] [0.3125 0.6875]
		z [0.11 0.111]
	)
)

define propeller2 actuator(
	color "black"
	size [0.1 0.001 0.015]

	block
	(
		points 4
		point[0] [0.25 0.5]
		point[1] [0.5 0.5]
		point[2] [0.5 0.75]
		point[3] [0.25 0.75]
		z [0.08 0.11]
	)
)

define propeller3 actuator(
	color "black"
	size [0.015 0.001 0.1]

	block
	(
		points 4
		point[0] [0.25 0.5]
		point[1] [0.5 0.5]
		point[2] [0.5 0.75]
		point[3] [0.25 0.75]
		z [0.08 0.11]
	)
)

define square_cap actuator(
	color "dark gray"
	size [0.07 0.07 0.004]

	block
	(
		points 4
		point[0] [0.25 0.5]
		point[1] [0.5 0.5]
		point[2] [0.5 0.75]
		point[3] [0.25 0.75]
		z [0.113 0.117]
	)
)

define tail actuator(
	color "DarkSeaGreen4"
	size [0.3 0.14 0.1]

	block
	(
		points 4
		point[0] [0 0.125]
		point[1] [0 0.5]
		point[2] [-0.6 0.335]
		point[3] [-0.6 0.290]
		z [0.0 0.1]
	)
)

define tail_top actuator(
	color "DarkSeaGreen4"
	size [0.075 0.0475 0.1]

	block
	(
		points 4
		point[0] [0 -0.00625]
		point[1] [0 0.1]
		point[2] [-0.15 0.065555]
		point[3] [-0.15 0.028195]
		z [0.0 0.1]
	)
)

define side_flap actuator(
	color "DarkSeaGreen4"
	size [0.15 0.04 0.01]

	block
	(
		points 4
		point[0] [0.5 0.1]
		point[1] [0.75 0.1]
		point[2] [0.75 0.4]
		point[3] [0.5 0.4]
		z [0.0 0.01]
	)
)

define gun actuator(
	color "black"
	size [0.1 0.02 0.02]	

	block
	(
		points 4
		point[0] [0.3125 0.5625]
		point[1] [0.75 0.5625]
		point[2] [0.75 0.6875]
		point[3] [0.3125 0.6875]
		z [0.0 0.02]
	)
)

define gun2 actuator(
	color "black"
	size [0.06 0.02 0.02]	

	block
	(
		points 4
		point[0] [0.3125 0.5625]
		point[1] [0.75 0.5625]
		point[2] [0.75 0.6875]
		point[3] [0.3125 0.6875]
		z [0.0 0.02]
	)
)

define dots actuator(
	color "black"
	size [0.02 0.02 0.02]	

	block
	(
		points 4
		point[0] [0.3125 0.5625]
		point[1] [0.75 0.5625]
		point[2] [0.75 0.6875]
		point[3] [0.3125 0.6875]
		z [0.0 0.02]
	)
)



define simple_robot position(
	odom_error [0.03 0.03 0.00 0.05]
	localization "odom"
	size [0.4 0.3 0.2]
	gui_nose 1
	drive "diff"
	color "DarkSeaGreen4"

	laser(pose [0.23 0.0 -0.2 0.0])
	rhomboid(pose [-0.057 0.0 0.0 0.0])
	screen(pose [0.038 0.0 0.0 0])
	junction_top(pose [-0.057 0.0 0.0 0.0])
	junction_low(pose [-0.4125 -0.025 -0.1 0.0])
	propeller1(pose [-0.057 0.0 0.0 60])
	propeller1(pose [-0.057 0.0 0.0 -60])
	propeller2(pose [-0.4125 -0.0405 -0.1 0])
	propeller3(pose [-0.4125 -0.0405 -0.366 0])
	tail(pose [-0.3 0.0 -0.2 0.0])
	tail_top(pose [-0.4125 0.0 -0.1 0.0])
	square_cap(pose [-0.057 0.0 0.0 0])
	side_flap(pose [-0.08 0.17 -0.1 90])
	side_flap(pose [-0.08 -0.17 -0.1 90])
	gun(pose [-0.07 0.228 -0.12 0])
	gun(pose [-0.07 -0.228 -0.12 0])
	gun2(pose [-0.07 0.18 -0.12 0])
	gun2(pose [-0.07 -0.18 -0.12 0])
	dots(pose [0.04 0.075 -0.03 0])
	dots(pose [0.04 0.075 -0.07 0])
	dots(pose [0.04 0.075 -0.11 0])
	dots(pose [0.04 0.075 -0.15 0])
	dots(pose [0.04 0.075 -0.19 0])
	dots(pose [0.04 -0.075 -0.03 0])
	dots(pose [0.04 -0.075 -0.07 0])
	dots(pose [0.04 -0.075 -0.11 0])
	dots(pose [0.04 -0.075 -0.15 0])
	dots(pose [0.04 -0.075 -0.19 0])



	#base	
	block
	(
		points 10
		point[0] [0.75 0.25]
		point[1] [0.75 0.375]
		point[2] [1.5 0.5]
		point[3] [1.5 0.75]
		point[4] [0.75 0.875]
		point[5] [0.75 1]
		point[6] [0 1.25]
		point[7] [-0.25 0.875]
		point[8] [-0.25 0.375]
		point[9] [0 0]
		z [0 0.2]
	)
)








