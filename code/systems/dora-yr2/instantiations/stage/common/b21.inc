# a Pioneer 2 or 3 in standard configuration
define b21 position
(
  # actual size
  size [0.52 0.52]

  # the pioneer's center of rotation is offset from its center of area
  origin [0 0 0]

  # draw a nose	on the robot so we can see which way it points
  gui_nose 1
  gui_boundary 0	

  # estimated mass in KG
  mass 60.0 
	
  # this polygon approximates the shape of a pioneer
  polygons 1
  polygon[0].points 24
  polygon[0].point[0] [ -0.248 -0.033 ]
  polygon[0].point[1] [ -0.231 -0.096 ]
  polygon[0].point[2] [ -0.198 -0.152 ]
  polygon[0].point[3] [ -0.152 -0.198 ]
  polygon[0].point[4] [ -0.096 -0.231 ]
  polygon[0].point[5] [ -0.033 -0.248 ]
  polygon[0].point[6] [ 0.033 -0.248 ]
  polygon[0].point[7] [ 0.096 -0.231 ]
  polygon[0].point[8] [ 0.152 -0.198 ]
  polygon[0].point[9] [ 0.198 -0.152 ]
  polygon[0].point[10] [ 0.231 -0.096 ]
  polygon[0].point[11] [ 0.248 -0.033 ]
  polygon[0].point[12] [ 0.248 0.033 ]
  polygon[0].point[13] [ 0.231 0.096 ]
  polygon[0].point[14] [ 0.198 0.152 ]
  polygon[0].point[15] [ 0.152 0.198 ]
  polygon[0].point[16] [ 0.096 0.231 ]
  polygon[0].point[17] [ 0.033 0.248 ]
  polygon[0].point[18] [ -0.033 0.248 ]
  polygon[0].point[19] [ -0.096 0.231 ]
  polygon[0].point[20] [ -0.152 0.198 ]
  polygon[0].point[21] [ -0.198 0.152 ]
  polygon[0].point[22] [ -0.231 0.096 ]
  polygon[0].point[23] [ -0.248 0.033 ]

  # differential steering model
  drive "diff"
) 

