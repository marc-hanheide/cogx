                                                                   
                                             
(define (problem cogxtask)

(:domain dt-cogx)

(:objects  robot_4_pt_spatial_data - robot
           place_1_pt_spatial_data - place
)

(:init  (connected place_0_pt_spatial_data place_1_pt_spatial_data)
        (is-in robot_4_pt_spatial_data place_0_pt_spatial_data)
)

(:goal  (and  ))
(:metric maximize (reward ))

)
