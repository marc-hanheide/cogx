define cogxp3_sonar ranger
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
  sview[0] [0 5.0 15]
  sview[1] [0 5.0 15]
  sview[2] [0 5.0 15]
  sview[3] [0 5.0 15]
  sview[4] [0 5.0 15]
  sview[5] [0 5.0 15]
  sview[6] [0 5.0 15]
  sview[7] [0 5.0 15]

  # define the size of each transducer [xsize ysize] in meters
  ssize[0] [0.01 0.05]
  ssize[1] [0.01 0.05]
  ssize[2] [0.01 0.05]
  ssize[3] [0.01 0.05]
  ssize[4] [0.01 0.05]
  ssize[5] [0.01 0.05]
  ssize[6] [0.01 0.05]
  ssize[7] [0.01 0.05]
)

# a Pioneer  in standard configuration
define cogxp3 position
(
  # actual size
  size [0.44 0.33]

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.04 0.0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1
  gui_boundary 0	

  # estimated mass in KG
  mass 15.0 
	
  # this polygon approximates the shape of a pioneer
  polygons 1
  polygon[0].points 8
  polygon[0].point[0] [  0.23  0.05 ]
  polygon[0].point[1] [  0.15  0.15 ]
  polygon[0].point[2] [ -0.15  0.15 ]
  polygon[0].point[3] [ -0.23  0.05 ]
  polygon[0].point[4] [ -0.23 -0.05 ]
  polygon[0].point[5] [ -0.15 -0.15 ]
  polygon[0].point[6] [  0.15 -0.15 ]
  polygon[0].point[7] [  0.23 -0.05 ]
  polygon[0].filled 1

  # differential steering model
  drive "diff"

  # use the sonar array defined above
  cogxp3_sonar()  
) 

