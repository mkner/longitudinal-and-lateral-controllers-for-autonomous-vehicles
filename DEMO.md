
### Short Demo Clips

## Reverse Test

Negative velocity control signal triggers reverse.
Zero puts on the brakes.


![rreverse test](demo/rev_test.gif)



## Worst Case Scenarios

Simulates a failure with the desired velocity signal from the local planner,  
or internal to the longitudinal controller (before the low-level control and acuators)
where the signal is stuck 90% above where it should be. The modified Stanley controller
does remarkable job at repeatedly getting back on course after swerving around and at  
some points over 10 meters off track in lateral directions. Finishes with a CTE of only  
1.74 meters. It could have been worse! 
