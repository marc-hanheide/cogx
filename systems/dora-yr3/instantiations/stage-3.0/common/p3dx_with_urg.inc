define p3dx_sonar ranger
(
  scount 8

  # define the pose of each transducer [xpos ypos heading]
  spose[0] [ 0.075 0.130 90 ]
  spose[1] [ 0.115 0.115 50 ]
  spose[2] [ 0.150 0.080 30 ]
  spose[3] [ 0.170 0.025 10 ]
  spose[4] [ 0.170 -0.025 -10 ]
  spose[5] [ 0.150 -0.080 -30 ]
  spose[6] [ 0.115 -0.115 -50 ]
  spose[7] [ 0.075 -0.130 -90 ]
		
  # define the field of view of each transducer [range_min range_max view_angle]
  sview [0 5.0 15]
  
  # define the size of each transducer [xsize ysize] in meters
  ssize [0.01 0.05]
)

# a Pioneer  in standard configuration
define p3dx position
(
  # actual size
  size [0.44 0.33 0.215]

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.04 0.0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 15.0 
	
  # this polygon approximates the shape of a pioneer
  block (
  	points 8
  	point[0] [  0.23  0.05 ]
  	point[1] [  0.15  0.15 ]
  	point[2] [ -0.15  0.15 ]
  	point[3] [ -0.23  0.05 ]
  	point[4] [ -0.23 -0.05 ]
  	point[5] [ -0.15 -0.15 ]
  	point[6] [  0.15 -0.15 ]
  	point[7] [  0.23 -0.05 ]
  	z [0.05 0.215]
  )
  # differential steering model
  drive "diff"

  color "red"

  # use the sonar array defined above
  p3dx_sonar()  
) 


define urg_04LX laser
(
  range_max 5.6
	#manual says radians, but only degrees appear to work
  # fov 4.1887902
  fov 240
  samples 654

  color "blue"
  size [ 0.14 0.14 0.14]	
)




# create a robot
define p3dx_with_urg p3dx
(
  urg_04LX( pose [0.14 0 0.0 0])
)

# create a robot
define p3dx_with_urg_and_blobfinder p3dx_with_urg
(
 actuator
 (
  # actuator properties
  pose [0.0 0 0.90 0]
  type "rotational"
  #axis [0 0 1]

  min_position -3.14
  max_position 3.14
  #max_speed 0.5
  max_speed 60.0
  color "green"
  size [ 0.32 0.19 0.19 ]
  blobfinder( 
    colors_count 6 
    colors [ "red" "blue" "green" "cyan" "yellow" "magenta" ]  
    range 2.0
    #pan 0.0  
 )
) 
  

)

