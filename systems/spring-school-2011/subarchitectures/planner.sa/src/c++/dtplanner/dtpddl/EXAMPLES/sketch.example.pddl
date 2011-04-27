
True state:

 R1 = Library

 R2 = Office

 R3 = Kitchen

The corn-flakes are located in the kitchen.


PLAN :::
--------

CP (Continual Planning) :: move-to-connected-room Hall R1

CP :: foreground_model Corn-flakes S1 

CP :: explore-room R1 (given Action "explore-room" results in an
                       observation, the POMDP planner is invoked )

CP -- Posts a goal ;; i.e., CP plan need to know the truth value of wid.-loc.
   :: expecting [TRUE == (widget-location Corn-flakes R1); \theta > .8]

POMDP * * explore-room R1

POMDP * [NA] (i.e., there is no perception in the observation)

POMDP * * foreground_model Empty S1 

POMDP * * foreground_model Chef S1 

POMDP * * explore-room R1

POMDP * [NA]

POMDP * * foreground_model Empty S1 

POMDP * * foreground_model Desktop S1 

POMDP * * explore-room R1

POMDP * [NA]

POMDP * * foreground_model Empty S1 

POMDP * * foreground_model Bookshelf S1 

POMDP * [(observed_model_at_room Bookshelf R1)]

POMDP *	Unlikely -- [TRUE == (widget-location Corn-flakes R1); \theta > .8]

POMDP ** Returns control to CP

CP :: move-to-connected-room R1 Hall

CP :: move-to-connected-room Hall R2

CP :: foreground_model Empty S1 

CP :: foreground_model Corn-flakes S1 

CP :: explore-room R2 (given Action "explore-room" results in an
                       observation, the POMDP planner is invoked )

CP -- Posts a goal 
   :: expecting [TRUE == (widget-location Corn-flakes R2); \theta > .8]

POMDP * * explore-room R2

POMDP * [NA]

POMDP * * foreground_model Empty S1 

POMDP * * foreground_model Desktop S1 

POMDP * [(observed_model_at_room Desktop R2)]

POMDP *	Unlikely -- [TRUE == (widget-location Corn-flakes R2); \theta > .8]

POMDP ** Returns control to CP

CP :: move-to-connected-room R2 Hall

CP :: move-to-connected-room Hall R3

CP :: foreground_model Empty S1 

CP :: foreground_model Corn-flakes S1 

CP :: explore-room R3 (given Action "explore-room" results in an
                       observation, the POMDP planner is invoked )

CP -- Posts a goal 
   :: expecting [TRUE == (widget-location Corn-flakes R3); \theta > .8]

POMDP * * explore-room R3

POMDP * [NA]

POMDP * * explore-room R3

POMDP * [NA]

POMDP * * foreground_model Empty S1 

POMDP * * foreground_model Chef S1 

POMDP * * explore-room R3

POMDP * [(observed_model_at_room Chef R3)]

POMDP * * explore-room R3

POMDP * [(observed_model_at_room Chef R3)]

POMDP * * foreground_model Empty S1 

POMDP * * foreground_model Corn-flakes S1 

POMDP * * explore-room R3

POMDP * [NA]

POMDP * * explore-room R3

POMDP * [(widget-location Corn-flakes R3)]

POMDP * * explore-room R3

POMDP * [(widget-location Corn-flakes R3)]

POMDP * * explore-room R3

POMDP * [NA]

POMDP * * explore-room R3

POMDP * [(widget-location Corn-flakes R3)]

POMDP *	[TRUE == (widget-location Corn-flakes R3); \theta > .8]

POMDP * * [SUGGESTS; (commit__widget_location Corn-flakes R3)]

CP :: commit__widget_location Corn-flakes R3


