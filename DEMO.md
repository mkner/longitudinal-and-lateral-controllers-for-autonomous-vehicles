
## Short Demo Clips

### Testing adverse and unexpected autonomous system problems

### Reverse 

Simulates a problem with the vehicle putting iteself into reverse when  
it was given a forward motion from the local planner. Tests ability of  
the controllers to recover from suddenly being put into reverse while still   
receiving the forward waypoint stream for position. The negative of  
velocity control signal triggers reverse. A zero velocity signal puts on  
the  brakes. Then the car puts itself back into forward gear.  




![rreverse test](demo/rev_test.gif)



### Target velocity 

Simulates a failure with the desired velocity signal from the local planner,  
or internal to the longitudinal controller (before the low-level control and acuators)  
where the signal is stuck 90% above where it should be. The modified Stanley  
controller does remarkable job at repeatedly getting back on course after swerving  
around and at some points over 18 meters off path in lateral directions. Recovers well  
from bouncing off a fence. Finishes with a CTE of only  1.74 meters and an average  
CTE of 1.84 meters. It could have been worse! Much worse!


![rreverse test](demo/vel90over.gif)
