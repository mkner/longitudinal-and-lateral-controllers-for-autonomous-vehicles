#
# local planner
#mk
#v0.08 
#

import numpy as np
import numpy.linalg as nla

import csv


class Path_generator(object):
    
    def __init__(self, waypoints_file, resolution = None):
 
        # load waypoints from file into array
    
        self.waypoints_file = waypoints_file #WAYPOINTS_FILENAME 
        self.waypoints   = None
        
        with open(self.waypoints_file) as waypoints_file_handle:
            wp_list = list(csv.reader(waypoints_file_handle, 
                                        delimiter=',',
                                        quoting=csv.QUOTE_NONNUMERIC))
        
        self.waypoints = np.array(wp_list)

        #print("waypoints LEN = " + str(len(self.waypoints)))
              
        self.new_waypoints = [] # holds new batch of higher res wps
        
        self.wp_interp      = []    # interpolated values 
                               # (rows = waypoints, columns = [x, y, v])
        self.wp_interp_hash = []    # hash table which indexes waypoints
                               # to the index of the waypoint in wp_interp
        self.interp_counter = 0    # counter current interpolated point index
    
        self.closest_index = 0
        
        self.closest_distance = 0
        
        self.new_index = 0 
        
        self.new_distance = 0
        
        self.wp_distances = []
        
        self.interp_resolution  = 0.01 # distance between interpolated points
        self.interp_lookahead_distance = 20 #20 # incomming stream lookahead in meters
        
        self._calc_interpolations() #one time only w/ resolution
        
    
    def set_lookahead_distance(self, lookahead = None):
        self.interp_lookahead_distance = lookahead
        
        
    def _calc_interpolations(self):

        waypoints = self.waypoints
        INTERP_DISTANCE_RES = self.interp_resolution
        
        wp_distances = []   
        
        for i in range(1, waypoints.shape[0]):
            wp_distances.append(
                    np.sqrt((waypoints[i, 0] - waypoints[i-1, 0])**2 +
                            (waypoints[i, 1] - waypoints[i-1, 1])**2))
        wp_distances.append(0)  # last distance is 0 because it is the distance
                                # from the last waypoint to the last waypoint
        
        # Linearly interpolate between waypoints and store in a list
        # along with their hash table
        wp_interp      = []    # interpolated values 
                               # (rows = waypoints, columns = [x, y, v])
        wp_interp_hash = []    # hash table which indexes waypoints
                               # to the index of the waypoint in wp_interp
        interp_counter = 0     # counter for current interpolated point index
        
        for i in range(waypoints.shape[0] - 1):
            
            # add original waypoint to interpolated waypoints list 
            # and append it to the hash table
            
            wp_interp.append(list(waypoints[i]))
            wp_interp_hash.append(interp_counter)   
            interp_counter+=1
            
            # interpolate to the next waypoint & calc the number of
            # points to interpolate based on the specified resolution and
            # incrementally add interpolated points until the next waypoint
            # is about to be reached
            
            num_pts_to_interp = int(np.floor(wp_distances[i] /\
                                         float(INTERP_DISTANCE_RES)) - 1)
            wp_vector = waypoints[i+1] - waypoints[i]
            wp_uvector = wp_vector / np.linalg.norm(wp_vector)
            for j in range(num_pts_to_interp):
                next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                wp_interp.append(list(waypoints[i] + next_wp_vector))
                interp_counter+=1
                
        # add last waypoint at the end
        wp_interp.append(list(waypoints[-1]))
        wp_interp_hash.append(interp_counter)   
        interp_counter+=1
        
        self.wp_distances=wp_distances
        self.wp_interp= wp_interp
        self.wp_interp_hash = wp_interp_hash
   

    def _calc_new_waypoints(self,cx,cy):
        
    # form a reduction subset of all waypoints to send to the controller
    # that are within some lookahead distance in front of and behind the vehicle
 
    # find closest waypoint index to car
    # increment the index from the previous one until the new distance calculations
    # start increasing & decrement the index 
    
    # the final index needs to be the closest point assuming that
    # the vehicle will always break out of instability points when there
    # are two indices with the minimum distance
    
        current_x = cx
        current_y = cy
        
        #print("current_x, current_y ",current_x, current_y)
        #print("self.closest_index ", self.closest_index)
        
        self.closest_distance = nla.norm(np.array([ 
            self.waypoints[self.closest_index, 0] - current_x, 
            self.waypoints[self.closest_index, 1] - current_y]))
        
        #print("self.closest_distance",self.closest_distance)
        
        self.new_distance = self.closest_distance
        self.new_index = self.closest_index

        while self.new_distance <= self.closest_distance:
        
            self.closest_distance = self.new_distance
            self.closest_index = self.new_index
            self.new_index += 1
        
            if self.new_index >= self.waypoints.shape[0]:  #end of path
                break

            self.new_distance = nla.norm(np.array([
                    self.waypoints[self.new_index, 0] - current_x,
                    self.waypoints[self.new_index, 1] - current_y]))
        
        self.new_distance = self.closest_distance
        self.new_index = self.closest_index
    
        while self.new_distance <= self.closest_distance:
        
            self.closest_distance = self.new_distance
            self.closest_index = self.new_index
            self.new_index -= 1
        
            if self.new_index < 0:  # beginning of path
                break
        
            self.new_distance = nla.norm(np.array([
                self.waypoints[self.new_index, 0] - current_x,
                self.waypoints[self.new_index, 1] - current_y]))

        # use closest index found to return the path that is one
        # waypoint behind and some lookahead waypoints in front
        # specified by a fixed lookahead distance 

        waypoint_subset_first_index = self.closest_index - 1
        
        if waypoint_subset_first_index < 0:
            waypoint_subset_first_index = 0
    
        waypoint_subset_last_index = self.closest_index
        total_distance_ahead = 0
        
        while total_distance_ahead < self.interp_lookahead_distance:
            
            total_distance_ahead += self.wp_distances[waypoint_subset_last_index]
            waypoint_subset_last_index += 1
            
            if waypoint_subset_last_index >= self.waypoints.shape[0]:
                waypoint_subset_last_index = self.waypoints.shape[0] - 1
                break
            
        self.new_waypoints = \
                self.wp_interp[self.wp_interp_hash[waypoint_subset_first_index]:\
                              self.wp_interp_hash[waypoint_subset_last_index] + 1]
            
   
    def get_wp_distances(self):
        return self.wp_distances
    
    def get_wp_interp(self):
        return self.wp_interp
    
    def get_all_waypoints(self):
        # waypoints already loaded on init
        return self.waypoints
    
    def get_new_waypoints(self,cx,cy):
        # need to feed in hash for the moment
        self._calc_new_waypoints(cx,cy) 
        return self.new_waypoints

    def get_wp_interp_hash(self):
        return self.wp_interp_hash
 
       
