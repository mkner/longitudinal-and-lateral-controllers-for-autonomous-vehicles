#!/usr/bin/env python3

#
#mk
#

CONTROLLER_VERSION="0.42"


#controller class for lateral & longitudinal controllers
#for  CARLA SDC  racetrack simulation


import utils
import numpy as np
import numpy.linalg as nla
import os
from collections import deque

### dont dump warnings
import warnings
warnings.filterwarnings('ignore')

# don't show pygame message on terminal
#not work either to not show pygame whatever message
# have to edit __init__.py for pygame library!
#from os import environ
#environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'


ANALYSIS_OUTPUT_FOLDER = os.getcwd() +'/analysis_output/'

print()
print()
print("Running controller version: "+str(CONTROLLER_VERSION)+"...")
print()
print("Initializing...")
print()

#class Controller2D(object):
    
class Controller2d(object):
    def __init__(self, waypoints):
        self.vars                = utils.Localvars() #object local variables
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        #mk new 0.42
        self._set_reverse        = False
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi
        #mk#
        # conversion constants
        self._to_mph=2.237 # meters per second to mph
        self._to_kmph=3.6 # meters per second to kmph
        # for post-run analysis
        self.analysis_file = ANALYSIS_OUTPUT_FOLDER+"analysis.txt"

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints
        
        # this returns stuff to send to simulator
    def get_commands(self):
        #mk new 0.42 reverse
        return self._set_throttle, self._set_steer, self._set_brake, self._set_reverse
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake
        
    #mk new 0.42 reverse    
    def set_reverse(self, input_reverse):
        #
        if input_reverse <=0: 
            self._set_reverse = False
        else:
            self._set_reverse = True
        
    #### mk  ######################################
        
    def clamp_delta(self, angle,degree):
        max_angle_deg=degree #70.0
        rad2steer  = 180.0 / (max_angle_deg / np.pi)
        # scale delta to radians to [-1, 1]
        steering_scaled= rad2steer * angle
        # Clamp the steering command to bounds of simulator
        steering  = np.fmax(np.fmin(steering_scaled, 1.0), -1.0) 
        return(steering)
    
    def create_output_dir(self,output_folder):
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
            
    def mps2kmph(self, mps):
        # meters per second to mph
        return mps * self._to_kmph

    def mps2mph(self, mps):
        # meters per second to kmph
        return mps * self._to_mph
    
    def write_analysis_file(self):
        #mk
        self.create_output_dir(ANALYSIS_OUTPUT_FOLDER)
        #file_name = os.path.join(ANALYSIS_OUTPUT_FOLDER, "analysis-data-v"\
        #                         +str(CONTROLLER_VERSION)+".txt")
        self.analysis_file = os.path.join(ANALYSIS_OUTPUT_FOLDER, "analysis_data-v"\
                                 +CONTROLLER_VERSION+".txt")
        #print()
        print("Saving analysis data...")
        
        #print(self.analysis_file)
        print()
        with open(self.analysis_file, 'w') as output_file: 
            for i in range(len(self.vars.time_data)):
                output_file.write('%3.3f, %3.3f, %3.3f,%3.3f, %3.3f, %3.3f, %3.3f,%3.3f, %3.3f,%3.3f, %3.3f, %3.3f, %3.3f, %3.3f\n'%\
                  ( self.vars.time_data[i], self.vars.cx_data[i],self.vars.cy_data[i],\
                    self.vars.yaw_data[i], self.vars.v_actuals[i], self.vars.v_tracking_data[i],\
                    self.vars.v_errors[i], self.vars.v_averages[i],\
                    self.vars.nx_data[i],self.vars.ny_data[i],\
                    self.vars.ct_error_data[i], self.vars.ct_error_avg_data[i],\
                    self.vars.psi_data[i], self.vars.delta_data[i]) )
                    
    
    
    # main interation loop called update_controls if flagged to run
    
    def update_controls(self):
        
        ######################################################
        # retreive path plan & velocity profile
        ######################################################
        
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steering_output    = 0
        brake_output    = 0
    
        ############################################
        ###  declarations for persistant vars
        ############################################

        ### local
        self.vars.create('i',int(0))
        self.vars.create('delta_prev',float(0))
        self.vars.create('dumped_analysis',False)
        #
        # for runtime displays
        #
        self.vars.create('current_cte',float(0)) 
        self.vars.create('current_cte_avg',float(0)) 
        self.vars.create('current_v_err',float(0)) 
        self.vars.create('current_v_avg',float(0)) 
        
        
        ### analysis
        
        # when
        self.vars.create('time_data',deque())
        
        # pose: now, where & which way
        
        self.vars.create('cx_data',deque()) # current x pos
        self.vars.create('cy_data',deque()) # current y pos
        self.vars.create('yaw_data',deque()) # heading angle
        
        # where to and how
        
        ## longitudinal ##
        self.vars.create('v_actuals',deque()) # actual velocity
        self.vars.create('v_tracking_data',deque()) # requested velocity
        self.vars.create('v_errors',deque()) # v error for PID input
        self.vars.create('v_averages',deque()) # average v so far
         
        ## lateral ##
        self.vars.create('nx_data',deque()) # nearest x pos
        self.vars.create('ny_data',deque()) # nearest y pos
        self.vars.create('ct_error_data',deque()) # stanley crosstrack
        self.vars.create('psi_data',deque()) #  not pressure, greek!
        self.vars.create('delta_data',deque()) # steering angle
        # new vers v0.34
        self.vars.create('ct_error_avg_data',deque()) # running avg crosstrack
    

        # ok to run main control loop??
        
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
            Controller iteration code block.

            Controller Feedback Variables:
                x               : Current X position (meters)
                y               : Current Y position (meters)
                yaw             : Current yaw pose (radians)
                v               : Current velocity (abs can be speed) (meters per second)
                t               : Current time (seconds)
                v_desired       : Current desired speed (meters per second)
                                  (Computed as the speed to track at the
                                  closest waypoint to the vehicle.)
                waypoints       : Current incoming stream of waypoints to track
                                  w/ velocity to track at each point (x,y)
                                  Format: [[x0, y0, v0],
                                           [x1, y1, v1],
                                           ...
                                           [xn, yn, vn]]
            
            Controller Output Variables:
                # these signals get sent to sim server
                throttle_output : Throttle output (0 to 1)
                steer_output    : Steer output (-1.22 rad to 1.22 rad)
                brake_output    : Brake output (0 to 1)
                
            """
        
        ######################################################
        # CONTROLLERS START
        ######################################################
        
            # note: these variables used below are reasignments (equates)
            # that have been already retreived 
            # at iteration in the loop, have global scope in this module

            """
            x               = self._current_x (meters)
            y               = self._current_y
            yaw             = self._current_yaw
            v               = self._current_speed
            v_desired       = self._desired_speed
            t               = self._current_timestamp
            waypoints       = self._waypoints
         
            """
        
        #########################################
        
        
        # get current waypoint finer resolution representation
        
        # (x,y,v) waypoints with velocity attribute only
        
        wpv=np.array(waypoints) #make an array [n x 3], thanks
        
        wp=wpv[:,:2] #(x,y) waypoints only

        cg=np.array([x,y]) # for realtime position
        
        velocity=v
        
        # test over throttled *here ###
        
        v_desired=v_desired#*5#1.60#*1.25#*2.5#1.75 #*1.60 # *5 #*2.5  #*2#1.60#*1.25#1.60# 1.90#1.50
        
      
        # begin terminal readouts
        
        print()
        #print("=============================")
        print()
        print("  time: "+'{0:3.2f}'.format(t)+"   i: "+str(self.vars.i)\
              + "   wps: "+str(len(wp))) #wpsz len waypoint buffer
        print()
       
        #print("  wp size: "+str(len(wp)))
        #print()
        
        ##############################################
        # LONGITUDINAL CONTROLLER                    #
        ##############################################

        #print("position (x,y): "+'{0:3.2f}'.format(x)+"  "+'{0:3.2f}'.format(y))
        print("  position: "+"("'{0:3.2f}'.format(x)+"),("+'{0:3.2f}'.format(y)+") "+" (x,y)")
        
        print()
        #print("  velocity: ", "  (" + str(int(self.mps2mph(v))) + ") mph "\
        #      + " (" + str(int(self.mps2kmph(v))) + ") kmph "\
        #      + " ("+str(int(v))+") mps")
            
        print("  velocity: ", "  (" + str(int(self.mps2kmph(v))) + ") kmph "\
              + " (" + str(int(self.mps2mph(v))) + ") mph "\
              + " ("+str(int(v))+") mps")
        # print("  velocity: ", "  (" + str(int(v*to_mph)) + ") mph "\
        #      + " (" + str(int(v*to_kmph)) + ") kmph "\
        #      + " ("+str(int(v))+") mps")
        print()
        
        ### PID - LONGITUDINAL CONTROLLER ###

        # from typical cruise control models some possible design contstraints
        # Rise time < 5 sec : Overshoot < 10 : Steady-state error < 2%
    
        # 0.03 for 30 fps & 0.02 for 20 fps?
        
        dt=0.03 #0.03 or whatever the time step inc is for this iter
        
        # set gains
        Kp=1
        Ki=1 #1.1 
        Kd=0.04
    
        
        #mk new in v0.42
        # ck here for negative v_desired
        # if < 0 then set in reverse gear
        # and use abs of the reference signal
        if (v_desired >= 0):
            reverse_output = False
        else:
            reverse_output = True
            v_desired = abs(v_desired)
           
        v_error = v_desired-v # v is velocity reading from car system state 
        # save for runtime display
        self.vars.current_v_err=v_error        
        
        if (len(self.vars.v_errors)) >=2: #watch bound in index
            i_error = sum(self.vars.v_errors) * dt # may not need all these
            d_error = (self.vars.v_errors[-1] - self.vars.v_errors[-2])#changed 10/8/20 / dt     
        else:
            i_error = 0
            d_error = 0
        

        ## PID 
        
        throttle_pid_out=(Kp*v_error)+(Ki*i_error*dt)+((Kd*d_error)/dt)

        # brake now used
        if (throttle_pid_out < 0):
           ## no brake rem out to test
           brake_output = -throttle_pid_out
           throttle_output=0 # lay off the pedal
          # print();print(">>>>>>>braking at : "+str(brake_output))
        else:
            brake_output=0 # no braking
            #  clamp throttle signal & set
            throttle_output=np.clip(throttle_pid_out,0,1)
            
      
        #######################################
        # store runtime data
        #######################################
        
        self.vars.time_data.append(t) # save running velocity errors
        
        # pid velocity stuff
        self.vars.v_actuals.append(v) # current velocity from "sensors"
        self.vars.v_tracking_data.append(v_desired) # tracking velocity provided
        self.vars.v_errors.append(v_error) # save running velocity errors
        self.vars.v_averages.append(np.average(self.vars.v_actuals)) # 
        
        ######################################################################

        # readouts to terminal
        #*here
        v_avg = np.average(self.vars.v_actuals)
        self.vars.current_v_avg = v_avg # for runtime display
        
        print("  avg velocity:"," ("+str(int(self.mps2kmph(v_avg)))+") kmph  "\
               + "(" + str(int(self.mps2mph(v_avg))) + ") mph  "\
                   + "("+str(int(v_avg))+")"+" mps")
        print()
        
        #print()
        
        #print("  velocity error:",'{0:3.2f}'.format(v_error)," ("+str(int(self.mps2mph(v_error)))+ ") mph")
        print("  velocity error:"," ("+str(int(self.mps2kmph(v_error)))+") kmph  "\
               + "(" + str(int(self.mps2mph(v_error))) + ") mph  "\
                   + "("+str(int(v_error))+")"+" mps")
        
        verr_max=np.max(self.vars.v_errors) #print needs it this way, don't ask
        
        print()
        #print("  max velocity error:",'{0:3.2f}'.format(verr_max)," ("+str(int(self.mps2mph(verr_max)))+ ") mph")
        
        print("  max velocity error:"," ("+str(int(self.mps2kmph(verr_max)))+") kmph  "\
               + "(" + str(int(self.mps2mph(verr_max))) + ") mph  "\
                   + "("+str(int(verr_max))+")"+" mps")
        
        verr_min=np.min(self.vars.v_errors)
        #print("  min velocity error:",'{0:3.2f}'.format(verr_min)," ("+str(int(self.mps2mph(verr_max)))+ ") mph" )
        
        print("  min velocity error:"," ("+str(int(self.mps2kmph(verr_min)))+") kmph  "\
               + "(" + str(int(self.mps2mph(verr_min))) + ") mph  "\
                   + "("+str(int(verr_min))+")"+" mps")
        
        # for str
        verr_avg_mps = np.average(self.vars.v_errors)
        verr_avg_mps_rnd=round(verr_avg_mps,2)
        verr_avg_kmph = self.mps2kmph(verr_avg_mps)
        verr_avg_kmph_rnd=round(verr_avg_kmph,2)
        verr_avg_mph = self.mps2mph(verr_avg_mps)
        verr_avg_mph_rnd=round(verr_avg_mph,2)
        print()
        print("  avg velocity error:", "("+str(verr_avg_kmph_rnd)+")" + " kmph "+\
              "("+str(verr_avg_mph_rnd) +")"+" mph " +\
              "("+str(verr_avg_mps_rnd)+ ")" + " mps ")
        
        #print("  avg velocity error:",'{0:3.2f}'.format(verr_avg)," ("+str(int(self.mps2mph(verr_avg)))+") mph") 
        
        #v_rel_err_percent=(np.abs(v_error)/v_desired)*100
     
        #print("  avg velocity error:"," ("+str(self.mps2mph(verr_avg)))+") mph  "\
            #   + "(" + str(self.mps2kmph(verr_avg)) + ") kmph  "\
           #        + "("+str(self.mps2mps(verr_avg)+")"+" mps")
        
            
        #print()
        #print("  relative velocity error:",'{0:3.2f}'.format(v_rel_err_percent)+" %")
        # relative error to what is expected velocity for tracking 
        #print("avg relative velocity error %:",'{0:3.2f}'.format(np.average(self.vars.v_rel_errors_percent))) 
        
        #print("set_throttle: ",'{0:3.2f}'.format(throttle_output))#str(vthrottled))
        
        ##############################################
        # LATERAL CONTROLLER                         #
        ##############################################  
        
        
        #########################################
        ### STANLEY -- LATERAL CONTROLLER 
        #########################################
        
        # length from front to rear axle, ie. wheelbase
        L=2.8 # 2.8 is bmw 520i not using 1.5 is too short for a real wheelbase
        lr = lf = L/2 # assuming cg is in middle between front & read axles
        
        #front ref point coord
        frp=np.array([cg[0]+lf*np.cos(yaw),cg[1]+lf*np.sin(yaw)])
        # rear ref point coord
        rrp=np.array([cg[0]-lr*np.cos(yaw),cg[1]-lr*np.sin(yaw)])
        
        #mk 0.42
        
        # set current reference point for vehicle
        
        #mk this is ok
        crp = frp
        if (velocity >= 0 and reverse_output == False): # or in reverse gear
             # lead with our nose
             crp=frp  # current ref point is front for stanley
        else:
             crp=rrp # in reverse, use back axle for "front" end
        
       
        #crp = frp
        
        # get the higher resolution waypoint data stream and use
        # j as offset in front of vehicle to START start scan range
        # into the path plan in case there is lag time or 1st waypoint
        # in stream behind vehicle, with this project you never know
        
        #mk TRY THIS!!!
        #j=0
        #j=20
        #j=25 # 10 is ~ 1 meter 25 is 2.5 meters
        #j=5
        j=5
        
        lookahead_points=wp[j:-1] # datastream can shrink at end of run
       
        # get distance to neareast waypoint from current reference point
        
        lookahead_distances=np.sqrt( (lookahead_points[:,0]-crp[0])**2 \
                                    + (lookahead_points[:,1]-crp[1])**2 )
        
        # get nearest waypoint
        nwp = lookahead_points[np.argmin(lookahead_distances)] 
        
        
        # OR THIS
        #ct_error = np.min(lookahead_distances)
       
        # get crosstrack error & angle
        ct_vector = crp - nwp # crosstrack vector from car nearest path point
        ct_error = nla.norm(ct_vector)
        
        ct_angle = np.arctan2(ct_vector[1],ct_vector[0])
   
        lp=lookahead_points #shorthand
        
        path_angle = np.arctan2(lp[len(lp)//4][1]-crp[1],lp[len(lp)//4][0]-crp[0])
        
        #path_angle = np.arctan2(wp[len(wp)//2][1]-crp[1],wp[len(wp)//2][0]-crp[0])
        
        # which side of path are we on to determine steering change direction
        path2crosstrack_angle = path_angle - ct_angle
        
        # expand numerical concept of zero
        # to avoid computational numerical instability for very
        # close intervals about zero
        # and to recuce over-oscillation of steering commands 
        # and other propagated artifacts
        # this depends on the the tuning of the controller equations
        # and the simulation, can be adjusted 
        
        #p2c_angle_bound = np.deg2rad(4) #  +/- degrees about zero 
        
        p2c_angle_bound = np.deg2rad(3) #  +/- degrees about zero 
        
        if (path2crosstrack_angle) > p2c_angle_bound:
             direction = -1
        elif (path2crosstrack_angle) < -p2c_angle_bound:
             direction = 1
        else: # centered enough so steering wheel no direction change
             direction = 0 # then keep steering steady where it was
        
        # get heading error psi 
        psi = path_angle - yaw
        psi = np.arctan2(np.sin(psi),np.cos(psi)) # bound check in [-pi,pi]
         
        # experimental
        # set Kv dynamically as function of velocity history
        # Kv min is minumum speed to reach before avgerages kick in
        # for extreme conditions where throttle stuck and out of control
        # for instance to see how can get enough reaction out of 
        # the controller
        
        #*here
        Kv_min= 10/self._to_mph # mph to mps 
        Kv = max( Kv_min, np.average(self.vars.v_actuals) )
        #Kv = min( Kv_min, np.average(self.vars.v_actuals) )
        
        Kps=0.05 # stanley proptional gain determined from field test
       # Kps = 0.10
        #Kps=1
        cte_term = direction * np.arctan((Kps * ct_error) / (Kv+velocity)) 
       
        if direction==0:# keep steady on course
            delta=self.vars.delta_prev
        else:  
            delta = psi + cte_term
            delta = np.arctan2(np.sin(delta),np.cos(delta)) # bound check in [-pi,pi]
        
        
        self.vars.delta_prev=delta # save this
        
        steering_output=delta
        
        print()
        #print("i="+str(self.vars.i))
        print("  crp (x,y): "+"( "'{0:3.1f}'.format(crp[0])+" , "+'{0:3.1f}'.format(crp[1])+" )",str(np.sign(crp)) )
        print("  nwp (x,y): "+"( "'{0:3.1f}'.format(nwp[0])+" , "+'{0:3.1f}'.format(nwp[1])+" )",str(np.sign(nwp)) )
        print()
        print("  yaw: ",  str(int(np.rad2deg(yaw)))+" (deg) ") #+str(round(yaw,2)) + " (rad)  "  ) 
        #print("  path angle: ",str(np.round(np.rad2deg(path_angle),2))+" (deg)")
        print("  path angle: ",str(int(np.rad2deg(path_angle)))+" (deg)")
        #print()
    
        #print("crosstrack angle (deg): ",str(np.round(np.rad2deg(ct_angle),2))+"") 
        #print("path to ct angle (deg): ",str(np.round(np.rad2deg(path2crosstrack_angle),2))+"")
        
        # right justify readout
        #print(f"{'  crosstrack angle:':<15}{np.round(np.rad2deg(ct_angle),2):>10}"+" (deg)")
        #print(f"{'path to ct angle (deg):':<15}{np.round(np.rad2deg(ct_angle),2):>10}")
        print()
        
        #print("velocity ",'{0:3.2f} '.format(v)+ " ("+str(int(v*3.6))+") kmph"\
        #      "  ("+str(int(v*2.237))+") mph") 
        #print()
        #print("crosstrack vector: "+str(np.round(ct_vector,2)),str(np.sign(ct_vector)) )
        #print()
        
        if path2crosstrack_angle >0:
            side = "   <<=  LEFT"
        elif path2crosstrack_angle <0:  
            side= "   =>> RIGHT"
        elif direction == 0:
            side= "  >> CENTER <<" # if within bounds for stable steering
        else:
            side="" #who knows - catch bizarre numerical error?!
            
        print("  crosstrack error: "+ str(round(ct_error,2)) + " "+side )
        print()
         
        
        # append earlier
        # analysis data dump doesnt save avg yet
        #*here
        
        # put sign in for which side so analysis plots shows variance 
        # on either side of trajectory path, then can also plot abs
        # to get magnitude only
        
        #self.vars.ct_error_data.append(ct_error)
        
        cte=np.sign(path2crosstrack_angle)*ct_error
        
        #self.vars.ct_error_data.append(np.sign(path2crosstrack_angle)*ct_error)
        self.vars.ct_error_data.append(cte)
        
        self.vars.current_cte = cte # save for runtime displays access
       
        #print("len ct error avg list "+str(len(self.vars.ct_error_data)))
        
        #save positive only, dont care which side, only magnitude for avg
        ct_error_avg=np.average(np.abs(self.vars.ct_error_data))
        self.vars.ct_error_avg_data.append(ct_error_avg) # write out also
        #mk
        self.vars.current_cte_avg = ct_error_avg # save for runtime displays access
        
        
        
        print("  avg crosstrack error: " + str(round(ct_error_avg,2)))
        
        print()
        print("  delta: " + str(int(np.round(np.rad2deg(delta))))+" (deg)" + \
              "   psi: " + str(int(np.round(np.rad2deg(psi))))+" (deg)" + \
              "   cte term: " + str(  np.round(cte_term) ) +" (deg)")#))
        print()
        
        #######################################
        # store runtime data
        #######################################
       
        # save lateral analysis data
        
        self.vars.cx_data.append(cg[0])
        self.vars.cy_data.append(cg[1])
        self.vars.nx_data.append(nwp[0])
        self.vars.ny_data.append(nwp[1])
        self.vars.yaw_data.append(yaw)
        
        self.vars.psi_data.append(psi)
        self.vars.delta_data.append(delta)
        
        ######################################################
        # set controller outputs for controller client to 
        # send signals to server
        ######################################################
        
        self.set_throttle(throttle_output)  # in percent (0 to 1)
        
        self.set_steer(steering_output)        # in rad (-1.22 to 1.22)
        #print("steering set to: " + str(np.round(self._set_steer,4) ))
        
        self.set_brake(brake_output)        # in percent (0 to 1)
        
        #mk new in 0.42
        self.set_reverse(reverse_output)
        
        # update iteration counter
        self.vars.i = self.vars.i+1
        
        
        # ck and stop run here?????
        
        # maybe set a #define for this later
        if len(wp)<110 and self.vars.dumped_analysis==False:  
             
            print();print("Simulation controller results momentarily... ");print()
            
            #mk THIS is moved into controller_client
            
            #print("-----------------------------------")
            #print("DUMPING ANALYSIS TO FILE HERE !!!! ")
            #print("-----------------------------------")
            #print()
            

        
        #########################################################
        ##     END LOOP    ######################################
        #########################################################
       
