
# Longitudinal and lateral controllers for autonomous vehicles

###Features PID and enhanced Stanley controllers
### Implementation runs on the CARLA simulator


### New in this release - v0.45

* A negative velocity control signal triggers the vehicle to put itself into reverse and  
the vehicle's CRP (Current Reference Point) for the Stanley lateral controller is set  
to the RRP (Rear Reference Point) instead of the normal FRP (Front Reference Point).

* Formatting improved for console readouts

* Some video clips of test runs included


### v0.44

- Better modularization of autonomous software system functions and capabilities

* Runtime display panels configuration options have been enhanced

* Can select which metrics of the runtime controls and state to display

* Velocity units can be set to mps, kmph or mph

* Size of the panels can be set to given dimensions

* Vertical or horizontal orientation of the panels an be specified for both controls and trajectory

* If a particular runtime display panel type is not set in options config then defaults are used








