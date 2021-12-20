#!/usr/bin/env python3

#
#mk 
#v8.12
#
# controller_client for carla simulator

# This work is licensed under the terms of the MIT license
# see <https://opensource.org/licenses/MIT>


from __future__ import print_function
from __future__ import division


MOD_VERSION=8.12 #8.12 path_generator works

# System level imports
import sys
import os
import argparse
import logging
import time
import math
import numpy as np
import csv

import time

#import matplotlib.pyplot as plt

#from controllers import controller2d

#longitudinal & lateral controllers in this module
import controllers 

import configparser 


#import path_generator now in local

import local_planner

# script level imports
sys.path.append(os.path.abspath(sys.path[0] + '/..'))


import runtime_display as rd #runtime display module
  
from carla            import sensor
from carla.client     import make_carla_client, VehicleControl
from carla.settings   import CarlaSettings
from carla.tcp        import TCPConnectionError
from carla.controller import utils


SAVE_TRAJECTORY=True# no silver platter

print()
print("Starting up controller client version "+str(MOD_VERSION)+"...")
print()


# simulation parameters

ITER_FOR_SIM_TIMESTEP  = 10     # no. iterations to compute approx sim timestep
WAIT_TIME_BEFORE_START = 15.00   # game seconds (time before controller start)
TOTAL_RUN_TIME         = 200.00 # game seconds (total runtime before sim end)
TOTAL_FRAME_BUFFER     = 300    # number of frames to buffer after total runtime
NUM_PEDESTRIANS        = 0      # total number of pedestrians to spawn
NUM_VEHICLES           = 0      # total number of vehicles to spawn
SEED_PEDESTRIANS       = 0      # seed for pedestrian spawn randomizer
SEED_VEHICLES          = 0      # seed for vehicle spawn randomizer


WEATHERID = {
    "DEFAULT": 0,
    "CLEARNOON": 1,
    "CLOUDYNOON": 2,
    "WETNOON": 3,
    "WETCLOUDYNOON": 4,
    "MIDRAINYNOON": 5,
    "HARDRAINNOON": 6,
    "SOFTRAINNOON": 7,
    "CLEARSUNSET": 8,
    "CLOUDYSUNSET": 9,
    "WETSUNSET": 10,
    "WETCLOUDYSUNSET": 11,
    "MIDRAINSUNSET": 12,
    "HARDRAINSUNSET": 13,
    "SOFTRAINSUNSET": 14,
}

#SIMWEATHER = WEATHERID["CLEARNOON"]     # set simulation weather

SIMWEATHER = WEATHERID["CLEARSUNSET"]  
 
PLAYER_START_INDEX = 1      # spawn index for player (keep to 1)

WAYPOINTS_FILENAME = 'racetrack_waypoints.txt'  # waypoint file to load
#DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0  # some distance from last position before
                                       # simulation ends                           
DIST_THRESHOLD_TO_LAST_WAYPOINT = 3.0                           

#INTERP_MAX_POINTS_PLOT

INTERP_MAX_POINTS_PLOT    = 7 # number of waypoints used for displaying
 
#now parameter to path generator                                            
#INTERP_LOOKAHEAD_DISTANCE = 20   # incomming stream lookahead in meters

#now parameter to path generator
#INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points

# controller output directory
RUNTIME_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) +\
                         '/runtime_output' #/controller_output/'

# not for unix
#RUNTIME_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) +\
#                          '\\controller_output\\'

##### TESTING MK ############


#path_generator = path_generator.Path_generator(WAYPOINTS_FILENAME)
#  OR local_planner.path_generator(...)

# local planer
# make a path high resolution waypoint generator

path_generator = local_planner.Path_generator(WAYPOINTS_FILENAME, \
                                              resolution=0.01)
#path_generator.set_lookahead_distance(10)#7)
path_generator.set_lookahead_distance(7)

##path_generator.set_resolution(0.01) # not implemented

#############################


def make_carla_settings(args):

    # carla settings object to set parameters for the simulation run
    
    settings = CarlaSettings()
    
    # dont make non-agent info requests if there 
    # are no pedestrians or other vehicles
    
    get_non_player_agents_info = False
    if (NUM_PEDESTRIANS > 0 or NUM_VEHICLES > 0):
        get_non_player_agents_info = True

    # base level settings
    settings.set(
        SynchronousMode=True,
        SendNonPlayerAgentsInfo=get_non_player_agents_info, 
        NumberOfVehicles=NUM_VEHICLES,
        NumberOfPedestrians=NUM_PEDESTRIANS,
        SeedVehicles=SEED_VEHICLES,
        SeedPedestrians=SEED_PEDESTRIANS,
        WeatherId=SIMWEATHER,
        QualityLevel=args.quality_level)
    return settings


class Timer(object):
    
    # timer to  calculate FPS, while the lap or seconds since lap is
    # begun? used to compute elapsed time

    def __init__(self, period):
        self.step = 0
        self._lap_step = 0
        self._lap_time = time.time()
        self._period_for_lap = period

    def tick(self):
        self.step += 1

    def has_exceeded_lap_period(self):
        if self.elapsed_seconds_since_lap() >= self._period_for_lap:
            return True
        else:
            return False

    def lap(self):
        self._lap_step = self.step
        self._lap_time = time.time()

    def ticks_per_second(self):
        return float(self.step - self._lap_step) /\
                     self.elapsed_seconds_since_lap()

    def elapsed_seconds_since_lap(self):
        return time.time() - self._lap_time


def get_current_pose(measurement):
    
    """
    get current x,y,yaw pose from the client measurements


    Args:
        measurement:  CARLA client measurements (from read_data())

    Returns: (x, y, yaw)
        x: x position in meters
        y: y position in meters
        yaw: yaw position in radians
        
    """
    x   = measurement.player_measurements.transform.location.x
    y   = measurement.player_measurements.transform.location.y
    yaw = math.radians(measurement.player_measurements.transform.rotation.yaw)

    return (x, y, yaw)


def get_start_pos(scene):
    
    """
    get vehicle start x,y, yaw pose from the scene

    Args:
        scene: CARLA scene object

    Returns: (x, y, yaw)
        x: X position in meters
        y: Y position in meters
        yaw: Yaw position in radians
        
    """
    x = scene.player_start_spots[0].location.x
    y = scene.player_start_spots[0].location.y
    yaw = math.radians(scene.player_start_spots[0].rotation.yaw)

    return (x, y, yaw)

###

def send_control_command(client, throttle, steer, brake, 
                         hand_brake=False, reverse=False):
    """
    
    send control command to simulator 
    
    Args:
        client: The CARLA client object
        throttle: Throttle command for the sim car [0, 1]
        steer: Steer command for the sim car [-1, 1]
        brake: Brake command for the sim car [0, 1]
        hand_brake: Whether the hand brake is engaged
        reverse: Whether the sim car is in the reverse gear
        
    """
    #
    
    control = VehicleControl()
    
    # clamp to bounds
    
    steer = np.fmax(np.fmin(steer, 1.0), -1.0)
    throttle = np.fmax(np.fmin(throttle, 1.0), 0)
    brake = np.fmax(np.fmin(brake, 1.0), 0)

    control.steer = steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = hand_brake
    control.reverse = reverse
    
    client.send_control(control)


def create_controller_output_dir(output_folder):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)


def store_trajectory_plot(graph, fname):
    
    # save graphic
    
    create_controller_output_dir(RUNTIME_OUTPUT_FOLDER)

    file_name = os.path.join(RUNTIME_OUTPUT_FOLDER, fname)
    
    graph.savefig(file_name)


def write_trajectory_file(x_list, y_list, v_list, t_list):
    create_controller_output_dir(RUNTIME_OUTPUT_FOLDER)
    #file_name = os.path.join(RUNTIME_OUTPUT_FOLDER, 'trajectory.txt')
    file_name = os.path.join(RUNTIME_OUTPUT_FOLDER, 'trajectory_data.txt')

    with open(file_name, 'w') as trajectory_file: 
        for i in range(len(x_list)):
            trajectory_file.write('%3.3f, %3.3f, %2.3f, %6.3f\n' %\
                                  (x_list[i], y_list[i], v_list[i], t_list[i]))

##############################################
# main() calls exec to kick off simualtion run
##############################################

def exec_waypoint_nav_demo(args):
    
    # connect to server and keep line open until...
    
    with make_carla_client(args.host, args.port) as client:
        print();print('CARLA client connected...')

        settings = make_carla_settings(args)

        # Now we load these settings into the server. The server replies
        # with a scene description containing the available start spots for
        # the player. Here we can provide a CarlaSettings object or a
        # CarlaSettings.ini file as string.
        scene = client.load_settings(settings)

        # Refer to the player start folder in the WorldOutliner to see the 
        # player start information
        player_start = PLAYER_START_INDEX

        # Notify the server that we want to start the episode at the
        # player_start index. This function blocks until the server is ready
        # to start the episode.
        
        print('Starting episode in %r...' % scene.map_name)
        client.start_episode(player_start)

        #############################################
        # Load Configuration parameters
        #############################################

        # Load configuration file options.cfg and  parse out the options
        
        # runtime_plotting - used as a flag to run plotting code or not
        
        # runtime_plotting_interval - how frequently the runtime plotting updates

        config = configparser.ConfigParser()
        config.read(os.path.join(
                os.path.dirname(os.path.realpath(__file__)), 'options.cfg'))         
        #demo_opt = config['Demo Parameters']
        sim_opt = config['Simulation Parameters']
        
        # Get options 
        show_runtime_displays = sim_opt.get('show_runtime_displays', 'true').capitalize()
        #enable_runtime_plot = enable_runtime_plot == 'True' # ok, but
        show_runtime_displays = (show_runtime_displays == 'True')  # logic

        runtime_plot_period = float(sim_opt.get('runtime_plotting_interval', 0))
        #print();print("plotting delay: "+str(runtime_plot_period))
        runtime_plot_timer = Timer(runtime_plot_period)
        
        #mk
        
        # selectively choose which runtime displays to generate
        # can make minimal or none, too many can slow down 
        # some systems depending on cpu/gpu capability
        
        #display_runtime_trajectory
        show_trajectory_display  = sim_opt.get('show_trajectory_display', 'true').capitalize()
        show_trajectory_display  = (show_trajectory_display == 'True')
        
        # display sizes are now selectable
        # portrait mode better for trajectory
        
        trajectory_panel_x_inches = sim_opt.getfloat('trajectory_panel_x_inches', 2)
        trajectory_panel_y_inches = sim_opt.getfloat('trajectory_panel_y_inches', 3)
       
        # all on or off
        show_control_display  = sim_opt.get('show_control_display', 'true').capitalize()
        show_control_display  = (show_control_display == 'True')
        
        # control panel can be vertical or horizontal

        control_display_orientation  = sim_opt.get('control_display_orientation', 'vertical')#.capitalize()
        #control_display_orientation  = sim_opt.get('control_display_orientation', 'horizontal')#.capitalize()
        
        """
        print()
        print("options.cfg orientation: ",control_display_orientation )
        print()
        input("Press Enter to continue...      
        """
        
        # now selectable
        # size of individual mini displays
        runtime_plot_x_inches = sim_opt.getfloat('runtime_plot_x_inches', 3)
        runtime_plot_y_inches = sim_opt.getfloat('runtime_plot_y_inches', 2.25)
       
        #show_runtime_throttle 
        show_runtime_throttle  = sim_opt.get('show_runtime_throttle', 'true').capitalize()
        show_runtime_throttle  = (show_runtime_throttle  == 'True')
       
        #show_runtime_brake
        show_runtime_brake  = sim_opt.get('show_runtime_brake', 'true').capitalize()
        show_runtime_brake  = (show_runtime_brake == 'True')
        
        #show_runtime_steering
        show_runtime_steering  = sim_opt.get('show_runtime_steering', 'true').capitalize()
        show_runtime_steering  = (show_runtime_steering == 'True')
        
        #runtime_velocity
        show_runtime_velocity_track = sim_opt.get('show_runtime_velocity_track', 'true').capitalize()
        show_runtime_velocity_track  = (show_runtime_velocity_track == 'True')
        
        #runtime_crosstrack error
        show_runtime_cte = sim_opt.get('show_runtime_cte', 'true').capitalize()
        show_runtime_cte  = (show_runtime_cte == 'True')
        
        #runtime_crosstrack error avg
        show_runtime_cte_avg = sim_opt.get('show_runtime_cte_avg', 'true').capitalize()
        show_runtime_cte_avg  = (show_runtime_cte_avg == 'True')
        
      
        #runtime_crosstrack error avg
        show_runtime_velocity_err= sim_opt.get('show_runtime_velocity_err', 'true').capitalize()
        show_runtime_velocity_err = (show_runtime_velocity_err == 'True')
        
        show_runtime_velocity_avg= sim_opt.get('show_runtime_velocity_avg', 'true').capitalize()
        show_runtime_velocity_avg = (show_runtime_velocity_avg == 'True')
        
        #default is mps, does not effect readouts, 
        velocity_display_units = sim_opt.get('velocity_display_units', 'mps')
        
        #wait to reposition panels, useful for video recording of simulation
        panels_repositioned = False
        
        #TESTING THIS 
        # throttle & brake overlapping
        show_runtime_throttle_brake = True
        
        #### END GET CONFIG ################33
        
        #############################################
        # load waypoints
        #############################################
           
        #mk ok 
        waypoints = None
        waypoints = path_generator.get_all_waypoints() #in init WAYPOINTS_FILENAME)
        
     
        #mk aok
        #wp_distances = path_generator.get_wp_distances()
        #mk ok
        #wp_interp = path_generator.get_wp_interp() #wp_inter
        #mk AOK 
        #wp_interp_hash = [] 
        #wp_interp_hash = path_generator.get_wp_interp_hash() #.wp_interp_hash
        
        #############################################
        # Instantiate Controller Object  
        #############################################
        
        controller = controllers.Controller2d(waypoints)

        #mk 
        # simulation average timestep & total frames
        num_iterations = ITER_FOR_SIM_TIMESTEP
        # why check a define for this?
        if (ITER_FOR_SIM_TIMESTEP < 1): # at least! one timestep
            num_iterations = 1
        
        #mk
        # get current current data from the simulator server
        # and calc starting time
        # then ACK since in synchronous mode
        
        measurement_data, sensor_data = client.read_data()
        sim_start_stamp = measurement_data.game_timestamp / 1000.0
        
        # since in sync mode, send control command to ACK OK
        # to continue to next iteration
  
        send_control_command(client, throttle=0.0, steer=0, brake=1.0)
       
        # approximate average timestep? based on some initial iterations
        
        ############################################################
        ####### INTERATION LOOP STARTS HERE ########################
        ############################################################
        
        sim_duration = 0
        
        for i in range(num_iterations):
            # get current measurement & perception data 
            measurement_data, sensor_data = client.read_data()
            # ACK to get more data for next iteration
            send_control_command(client, throttle=0.0, steer=0, brake=1.0)
            # calc when found last timestamp 
            if i == num_iterations - 1:
                sim_duration = (measurement_data.game_timestamp / 1000.0) \
                                - sim_start_stamp
                                 
        # use average simulation timestep to calc number of frames
        # until approximate end of the simulation
          
        SIMULATION_TIME_STEP = sim_duration / float(num_iterations)
        
        # show this?
        print("Apx simulation timestep: " + \
              str(SIMULATION_TIME_STEP))
            
        TOTAL_EPISODE_FRAMES = \
          int((TOTAL_RUN_TIME + WAIT_TIME_BEFORE_START)\
              / SIMULATION_TIME_STEP) \
                + TOTAL_FRAME_BUFFER

        
        #############################################
        # Initialization of run 
        #############################################
                   
        # get current pose, state...
        measurement_data, sensor_data = client.read_data()
        start_x, start_y, start_yaw = get_current_pose(measurement_data)
        send_control_command(client, throttle=0.0, steer=0, brake=1.0) #AOK
        
        # init runtime history lists
        x_history     = [start_x]
        y_history     = [start_y]
        yaw_history   = [start_yaw]
        time_history  = [0]
        velocity_history = [0]

        #############################################
        # create & setup simulation displays
        #############################################
        
        # so far... Panels are:   
        # 1) trajectory, 2) vehicle state & controler output signals & other

        if show_runtime_displays: 
            
            #create the main display panels 
 
            if show_trajectory_display:
                trajectory_display = rd.DisplayPanel(tk_title="Trajectory",\
                                    orientation=control_display_orientation)#'portrait')
            
            # if any of the state & controller signal 
            #mk use || ?
            if (show_runtime_throttle == True or \
                show_runtime_brake == True or \
                show_runtime_velocity_track == True or \
                show_runtime_steering == True or \
                show_runtime_cte == True or \
                show_runtime_velocity_avg == True or \
                show_runtime_velocity_err == True or \
                show_runtime_throttle_brake == True):
                
                # create the display panel
                control_display = rd.DisplayPanel(tk_title="Control Monitor",\
                                    orientation=control_display_orientation)

            # 2D surface map position trajectory display 

            # for trajectory panel cropping 
            PLOT_LEFT          = 0.1    # in fractions of figure width and height
            PLOT_BOT           = 0.1    
            PLOT_WIDTH         = 0.9#0.8
            PLOT_HEIGHT        = 0.8
            
            
            if show_trajectory_display: 
                
                trajectory_fig = trajectory_display.plot_new_dynamic_2d_figure(
                        title='Vehicle Trajectory',
                        figsize=(trajectory_panel_x_inches, trajectory_panel_y_inches),
                        edgecolor="black",
                        rect=[PLOT_LEFT, PLOT_BOT, PLOT_WIDTH, PLOT_HEIGHT])
                       
                # since UE4 uses left-handed coordinate system the x-axis
                # in the graph is reversed
                trajectory_fig.set_invert_x_axis() 
                
                # keep x to y spacing equal size                                  
                trajectory_fig.set_axis_equal()
        
                # waypoint markers
                trajectory_fig.add_graph("waypoints", window_size=waypoints.shape[0],
                                         x0=waypoints[:,0], y0=waypoints[:,1],
                                         linestyle="-", marker="", color='g')
                # trajectory markers
                trajectory_fig.add_graph("trajectory", window_size=TOTAL_EPISODE_FRAMES,
                                         x0=[start_x]*TOTAL_EPISODE_FRAMES, 
                                         y0=[start_y]*TOTAL_EPISODE_FRAMES,
                                         color=[1, 0.5, 0])
                # lookahead path
                trajectory_fig.add_graph("lookahead_path", 
                                         window_size=INTERP_MAX_POINTS_PLOT,
                                         x0=[start_x]*INTERP_MAX_POINTS_PLOT, 
                                         y0=[start_y]*INTERP_MAX_POINTS_PLOT,
                                         color=[0, 0.7, 0.7],
                                         linewidth=4)
                
                # starting position marker
                trajectory_fig.add_graph("start_pos", window_size=1, 
                                         x0=[start_x], y0=[start_y],
                                         marker=11, color=[1, 0.5, 0], 
                                         markertext="", marker_text_offset=0)
                # end position marker
                trajectory_fig.add_graph("end_pos", window_size=1, 
                                         x0=[waypoints[-1, 0]], 
                                         y0=[waypoints[-1, 1]],
                                         #marker="D", color='r', 
                                         marker="", color='k',
                                         markertext="", marker_text_offset=1)
                # Add car marker, what do you think its called?!!
                #trajectory_fig.add_graph("car", window_size=1, 
                                         #marker="s", color='b', markertext="", #markertext="Car",
                                        # marker_text_offset=1)
                # Add car marker, what do you think its called!!
                trajectory_fig.add_graph("car", window_size=1, 
                                         marker=".", color='r', markertext="", #markertext="Car",
                                         marker_text_offset=0)
    
   
            MINI_DISPLAY_SIZE=(runtime_plot_x_inches,runtime_plot_y_inches)
            
            #mk not "forward speed", it is velocity, 
            #can move or bounce backwards so has both magnitude (speed) and direction (+/-)
            
            if show_control_display:
                
                #mk *here 
                
                if show_runtime_velocity_track: 
                    
                    # fig size is PRESET IN CLASS AS 2,3
                    velocity_fig = control_display.plot_new_dynamic_figure(\
                            title='Velocity Tracking',figsize=MINI_DISPLAY_SIZE)
                           
                    velocity_fig.add_graph("velocity", 
                                                label="velocity", 
                                                window_size=TOTAL_EPISODE_FRAMES)
                    velocity_fig.add_graph("reference_signal", 
                                                label="reference_signal", 
                                                window_size=TOTAL_EPISODE_FRAMES)
                
                
                # show throttle signals 
                if show_runtime_throttle:
                    throttle_fig = control_display.plot_new_dynamic_figure(\
                                    title="Throttle", figsize=MINI_DISPLAY_SIZE )
                    throttle_fig.add_graph("throttle", 
                                          label="throttle", 
                                          window_size=TOTAL_EPISODE_FRAMES)
                    
                    
                # show brake signals 
                if show_runtime_brake:
                    brake_fig = control_display.plot_new_dynamic_figure(\
                                 title="Brake", figsize=MINI_DISPLAY_SIZE )
                    brake_fig.add_graph("brake", 
                                        label="brake", 
                                        window_size=TOTAL_EPISODE_FRAMES)
                
                #mk new
                # show throttle & brake both on one graph
                if show_runtime_throttle_brake:
                    throttle_brake_fig = control_display.plot_new_dynamic_figure(\
                                    title="Throttle & Brake", figsize=MINI_DISPLAY_SIZE )
                    throttle_brake_fig.add_graph("throttle", 
                                          label="throttle", 
                                          window_size=TOTAL_EPISODE_FRAMES)
                    throttle_brake_fig.add_graph("brake", 
                                        label="brake", 
                                        window_size=TOTAL_EPISODE_FRAMES)
                
                # velocity stuff here
                
                if show_runtime_velocity_err:
                    velocity_err_fig = control_display.plot_new_dynamic_figure(\
                                 title="Velocity Err", figsize=MINI_DISPLAY_SIZE )
                    velocity_err_fig.add_graph("v_err", 
                                        label="v_err", 
                                        window_size=TOTAL_EPISODE_FRAMES)
                    
                if show_runtime_velocity_avg:
                    velocity_avg_fig = control_display.plot_new_dynamic_figure(\
                                 title="Velocity Avg", figsize=MINI_DISPLAY_SIZE )
                    velocity_avg_fig.add_graph("v_avg", 
                                        label="v_avg", 
                                        window_size=TOTAL_EPISODE_FRAMES)    
        
                # show steering signals 
                if show_runtime_steering:
                    steer_fig = control_display.plot_new_dynamic_figure(\
                                 title="Steering", figsize=MINI_DISPLAY_SIZE )
                    steer_fig.add_graph("steer", 
                                          label="steer", 
                                          window_size=TOTAL_EPISODE_FRAMES)

                #if plot_crosstrack_error:
                if show_runtime_cte:
                   cte_fig = control_display.plot_new_dynamic_figure(\
                                 title="CTE", figsize=MINI_DISPLAY_SIZE )
                   cte_fig.add_graph("cte", 
                                        label="CTE", 
                                        window_size=TOTAL_EPISODE_FRAMES)
               
                #if plot_crosstrack_error running avg
                if show_runtime_cte_avg:
                   cte_avg_fig = control_display.plot_new_dynamic_figure(\
                                 title="CTE avg", figsize=MINI_DISPLAY_SIZE )
                   cte_avg_fig.add_graph("CTE_avg", 
                                        label="CTE_avg", 
                                        window_size=TOTAL_EPISODE_FRAMES)
        
       
        # if not show_runtime_displays:  #mk  fixed this hack
        #     trajectory_display._root.withdraw()
        #     control_display._root.withdraw()        
               
        #############################################
        # start frame iteration loop
        #############################################
        
        reached_the_end = False
        skip_first_frame = True
        
        #mk which reference point is used for these buitlins?
        # note this assumes the closest is the 1st index at 0
        # lateral controller can calculate this differently for CTE
        # and lookahead distance to start of trajectory incomming stream
       
        ################################################
        ###########B FRAME LOOP ########################
        ################################################
        
        # iterate over frames until end of waypoints 
        # or TOTAL_EPISODE_FRAMES reached 
        
        for frame in range(TOTAL_EPISODE_FRAMES):
            
            # update current pose, state,etc
            measurement_data, sensor_data = client.read_data()

            # update pose, timestamp 
            #mk note sim calls velocity the forward "speed"
            current_x, current_y, current_yaw = \
                get_current_pose(measurement_data)
                
            current_speed = measurement_data.player_measurements.forward_speed 
            current_timestamp = float(measurement_data.game_timestamp) / 1000.0

            # wait some initial time before starting the demo

            if current_timestamp <= WAIT_TIME_BEFORE_START:
                #### PAUSE HERE TO MOVE/ALIGN PANELS
                #mk 
                if (panels_repositioned == False):
                    print();print("Position panels now...")
                    input("Press Enter to continue...")
                    panels_repositioned = True
            
                send_control_command(client, throttle=0.0, steer=0, brake=1.0)
                continue
            else:
                current_timestamp = current_timestamp - WAIT_TIME_BEFORE_START
            
            # save runtime pose, state
            x_history.append(current_x)
            y_history.append(current_y)
            yaw_history.append(current_yaw)
            velocity_history.append(current_speed)
            time_history.append(current_timestamp) 
            
            #mk ok
            # get a new incomming stream of higher res waypoints
            new_waypoints = path_generator.get_new_waypoints( \
              current_x, current_y)
             
            #print("=======================")
            #print("len new waypoints:",len(new_waypoints))
            
            # send the controller the interpolated waypoint stream 
            # along with the pose, velocity, runtime stamp, etc
            
            controller.update_waypoints(new_waypoints)

            controller.update_values(current_x, current_y, current_yaw, 
                                     current_speed,
                                     current_timestamp, frame)
            
            
            # AND get back the latest control signals FROM the controller(s)
            
            controller.update_controls()
            #mk new
            #cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
            #mk new
            cmd_throttle, cmd_steer, cmd_brake, cmd_reverse = controller.get_commands()

            # skip the first frame when starting the controller 
            if skip_first_frame and frame == 0:
                pass
            else:
                
                #mk only if flag set, cleaner this way, 
                # no flashing panel startup as with deleting a tk root 
                
                if show_runtime_displays:
                    
                    if show_trajectory_display: 
                        trajectory_fig.roll("trajectory", current_x, current_y)
                        trajectory_fig.roll("car", current_x, current_y)
                    
                        # when plotting lookahead path use only 
                        # (INTERP_MAX_POINTS_PLOT amount of points) to
                        # decrease load when generating the runtime display
                        
                        new_waypoints = np.array(new_waypoints)
                        path_indices = np.floor(np.linspace(0, 
                                                        new_waypoints.shape[0]-1,
                                                        INTERP_MAX_POINTS_PLOT))
                        trajectory_fig.update("lookahead_path", 
                            new_waypoints[path_indices.astype(int), 0],
                            new_waypoints[path_indices.astype(int), 1],
                            new_colour=[0, 0.7, 0.7])
                    
                    ## advance displays ######################
                    
                    if show_control_display:
                        
                        # convert to units specified in config for velocity 
                        
                        if  (velocity_display_units == 'mph'):
                            # in mph
                            v_current = controller.mps2mph(current_speed)
                            v_desired = controller.mps2mph(controller._desired_speed)
                            current_v_avg =controller.mps2mph(controller.vars.current_v_avg)
                            current_v_err=controller.mps2mph(controller.vars.current_v_err)
                            
                        elif (velocity_display_units == 'kmph'):
                            v_current = controller.mps2kmph(current_speed)
                            v_desired = controller.mps2kmph(controller._desired_speed)
                            current_v_avg =controller.mps2kmph(controller.vars.current_v_avg)
                            current_v_err=controller.mps2kmph(controller.vars.current_v_err)
                        
                        else: #vehicle physics already in mps
                            v_current = current_speed  
                            v_desired = controller._desired_speed
                            current_v_avg =controller.vars.current_v_avg
                            current_v_err=controller.vars.current_v_err
                            
                
                        # velocity
                        if show_runtime_velocity_track:
                            velocity_fig.roll("velocity", 
                                               current_timestamp, 
                                               v_current)
                            
                            velocity_fig.roll("reference_signal", 
                                               current_timestamp, 
                                               v_desired)
                               
                        # throttle
                        if show_runtime_throttle:
                            throttle_fig.roll("throttle", current_timestamp, cmd_throttle)
                        
                        #brake
                        if show_runtime_brake:
                            brake_fig.roll("brake", current_timestamp, cmd_brake)
                        
                        #steering
                        if show_runtime_steering:
                            steer_fig.roll("steer", current_timestamp, cmd_steer)
                        
                        #ct error
                        current_cte=controller.vars.current_cte
                        if show_runtime_cte:
                            cte_fig.roll("cte", current_timestamp, current_cte)
                            
                        #ct error avg
                        current_cte_avg=controller.vars.current_cte_avg
                        if show_runtime_cte_avg:
                            cte_avg_fig.roll("CTE_avg", current_timestamp, current_cte_avg)
                       
                        if show_runtime_velocity_avg:
                            velocity_avg_fig.roll("v_avg", current_timestamp, current_v_avg)
                            
                        
                        if show_runtime_velocity_err:
                            velocity_err_fig.roll("v_err", current_timestamp, current_v_err)
                        
                        #*here  
                        if show_runtime_throttle_brake:
                        
                            throttle_brake_fig.roll("throttle", \
                                              current_timestamp, \
                                              cmd_throttle)

                            throttle_brake_fig.roll("brake", \
                                           current_timestamp, \
                                           cmd_brake)
                            
   
                # update runtime plot with refresh rate set in options config
                
                # why called "lap"?
                
                if show_runtime_displays and \
                   runtime_plot_timer.has_exceeded_lap_period():
                       
                    if show_trajectory_display == True:
                        trajectory_display.refresh()
                        
                    if show_control_display:  
                        if (show_runtime_throttle == True or \
                            show_runtime_brake == True or \
                            show_runtime_velocity_track == True or \
                            show_runtime_steering == True or \
                            show_runtime_cte == True or \
                            show_runtime_cte_avg == True or \
                            show_runtime_velocity_avg == True or \
                            show_runtime_velocity_err == True or \
                            show_runtime_throttle_brake == True):
                            
                            control_display.refresh()
                        
                    runtime_plot_timer.lap()
                   
            #############################################
            #  SEND updated controller signals to simulator
            #############################################

            # reverse can be added here
            #mk testing
            #cmd_reverse = True
            send_control_command(client,
                                 throttle=cmd_throttle,
                                 steer=cmd_steer,
                                 brake=cmd_brake,
                                 reverse=cmd_reverse)

            
            # end simulation if vehicle is withing
            # DIST_THRESHOLD_TO_LAST_WAYPOINT to the last waypoint,
           
            dist_to_last_waypoint = np.linalg.norm(np.array([
                waypoints[-1][0] - current_x,
                waypoints[-1][1] - current_y]))
            
            if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                reached_the_end = True
            if reached_the_end:
                break

        #############  END FRAME LOOP ###############
        #############################################
        
        #############################################
        # simulation run ENDED - start saving results
        #############################################

        if reached_the_end:
            print("Vehicle at end of path")
            print()
            print("Saving simulation runtime results...")
        else:
            print("Never reached end of path. Dump results to file anyway...")
            #". Writing to controller_output...")
       
        # stop the car
        send_control_command(client, throttle=0.0, steer=0.0, brake=1.0)
        # again
        send_control_command(client, throttle=0.0, steer=0.0, brake=1.0)
        
       
        #############################################
        # save runtime display graphics
        #############################################

        
        if (show_runtime_displays):
            # only save if plotted
            if show_trajectory_display: 
                store_trajectory_plot(trajectory_fig.fig, 'trajectory.png')
            
            if show_runtime_velocity_track:
                store_trajectory_plot(velocity_fig.fig, 'velocity_tracking.png')
            
            if show_runtime_throttle:
                store_trajectory_plot(throttle_fig.fig, 'throttle.png')
            
            if show_runtime_brake:
                store_trajectory_plot(brake_fig.fig, 'brake.png')
                
            if show_runtime_steering:
                store_trajectory_plot(steer_fig.fig, 'steer.png')
                
            if show_runtime_cte:
                store_trajectory_plot(cte_fig.fig, 'cte.png')
                
            if show_runtime_cte_avg:
                store_trajectory_plot(cte_avg_fig.fig, 'cte_avg.png')
                
            if show_runtime_velocity_avg:
              store_trajectory_plot(velocity_avg_fig.fig, 'velocity_avg.png')
              
            if show_runtime_velocity_err:
              store_trajectory_plot(velocity_err_fig.fig, 'velocity_err.png')
            
            if show_runtime_throttle_brake:
              store_trajectory_plot(throttle_brake_fig.fig, 'throttle_brake.png')


        #############################################
        # save runtime pose, etc results
        #############################################

        if ( SAVE_TRAJECTORY == True):
            
            write_trajectory_file(x_history, y_history, velocity_history, time_history)
            
            #mk
            #print("-----------------------------------")
            print()
            #print("Dumping analysis data to file... ")
            #print("-----------------------------------")
            # print()
            
            
            #############################################    
            # save other results for post-run analysis
            #############################################
            
            controller.vars.dumped_analysis=True
            
            controller.write_analysis_file()  

        
        #mk wait first before exit
        print()
        input("Press Enter to continue...")


################################## main() #################################

def main():
    """

    Args:
        -v, --verbose: print debug information
        --host: IP of the host server (default: localhost)
        -p, --port: TCP port to listen to (default: 2000)
        -a, --autopilot: enable autopilot
        -q, --quality-level: graphics quality level [Low or Epic]
        -i, --images-to-disk: save images to disk
        -c, --carla-settings: Path to CarlaSettings.ini file
    """
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Low',
        help='graphics quality level.')
    argparser.add_argument(
        '-c', '--carla-settings',
        metavar='PATH',
        dest='settings_filepath',
        default=None,
        help='Path to a "CarlaSettings.ini" file')
    args = argparser.parse_args()

    # Logging startup info
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = '_out/episode_{:0>4d}/{:s}/{:0>6d}'

    # Execute when server connection is established
    while True:
        try:
            exec_waypoint_nav_demo(args)
            ##print('Done')
            return

        except TCPConnectionError as error:
            logging.error(error)
            time.sleep(1)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nSimulation stop requested...')
        print('Halting now!');print()

