#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
import math
from scipy.integrate import simps
import pdb


class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._lookahead_distance = 2.0 # GITHUB
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
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True


        # # Find closest point in the track waypoints
            # closest_x = waypoints[0][0]
            # closest_y = waypoints[0][1]

            # for item in waypoints:
            #     waypoint_x = item[0]# waypoints[i][0]
            #     waypoint_y = item[1]#waypoints[i][1]
            #     distance = (waypoint_y - y / waypoint_x - x)

            #     current_distance = (closest_y - y / closest_x - x)

            #     if distance <= current_distance:
            #         closest_x = item[0]#waypoints[i][0]
            #         closest_y = item[1]#waypoints[i][1]

    def get_lookahead_index(self, lookahead_distance):
        # Initialize the index of the closest waypoint and the minimum distance found
        min_idx = 0
        min_dist = float("inf")

        # Loop through all waypoints to find the closest one to the current position
        for i in range(len(self._waypoints)):
            # Calculate the Euclidean distance between the current position and the waypoint
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,   # Difference in the x-coordinate
                    self._waypoints[i][1] - self._current_y])) # Difference in the y-coordinate
            
            # If this distance is smaller than the current minimum, update min_dist and min_idx
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        # Initialize total distance covered from the closest waypoint and the lookahead index
        total_dist = min_dist
        lookahead_idx = min_idx

        # Continue from the closest waypoint and find the waypoint that is at the lookahead distance
        for i in range(min_idx + 1, len(self._waypoints)):
            # If the total distance exceeds the lookahead distance, stop searching
            if total_dist >= lookahead_distance:
                break
            
            # Calculate the distance between the current waypoint and the previous waypoint
            total_dist += np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._waypoints[i-1][0],  # Difference in x-coordinates
                    self._waypoints[i][1] - self._waypoints[i-1][1]])) # Difference in y-coordinates
            
            # Update the lookahead index to the current waypoint
            lookahead_idx = i
        
        # Return the index of the waypoint that is at or just beyond the lookahead distance
        return lookahead_idx

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

    def get_commands(self):
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

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
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
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # DECLARE USAGE VARIABLES 

        # # Position, Velocity, Time
        # self.vars.create_var("x_prev", 0)
        # self.vars.create_var("y_prev", 0)
        # self.vars.create_var("v_prev", 0)
        # self.vars.create_var("yaw_prev", 0)
        # self.vars.create_var("t_prev", 0)
        # timesteps = 5

        # # Output Variables? -- not sure if i need this
        # self.vars.create_var("throttle_prev", 0)
        # self.vars.create_var("steer_prev", 0)
        # self.vars.create_var("breake_prev", 0)

        # # Longitudinal Controller
        # # Error values for PID
        # self.vars.create_var('v_error', 0.0) 
        # self.vars.create_var('v_error_prev', 0.0) 
        # self.vars.create_var('v_error_integral', 0.0) 

        # # PID Controller values, adjustable
        # self.vars.create_var('kp', 0.1)
        # self.vars.create_var('ki', 0.1)
        # self.vars.create_var('integrator_min', 0.0)
        # self.vars.create_var('integrator_max', timesteps)
        # self.vars.create_var('kd', 0.1)

        # # Lateral Controller
        # self.vars.create_var("heading_prev", 0)
        # self.vars.create_var("steering_min", -1.22) # in rads
        # self.vars.create_var("steering_max", 1.22)


        # PID Controller values - tuned
        self.vars.create_var('kp', 0.50)
        self.vars.create_var('ki', 0.30)
        self.vars.create_var('integrator_min', 0.0)
        self.vars.create_var('integrator_max', 10.0)
        self.vars.create_var('kd', 0.13)


        # Lateral Controllers Variables
        self.vars.create_var('kp_heading', 8.00)
        self.vars.create_var('k_speed_crosstrack', 0.00)
        self.vars.create_var('cross_track_deadband', 0.01)

        # Initial Values - updated each timestep
        self.vars.create_var('x_prev', 0.0)
        self.vars.create_var('y_prev', 0.0)
        self.vars.create_var('yaw_prev', 0.0)
        self.vars.create_var('v_prev', 0.0)
        self.vars.create_var('t_prev', 0.0)

        # Longitudinal Controller Variables
        self.vars.create_var('v_error', 0.0)
        self.vars.create_var('v_error_prev', 0.0)
        self.vars.create_var('v_error_integral', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # LONGITUDINAL CONTROLLER

            # PID in the time domain
            self.vars.v_error           = v_desired - v
            self.vars.v_error_integral += self.vars.v_error * (t - self.vars.t_prev)
            v_error_rate_of_change      = (self.vars.v_error - self.vars.v_error_prev) / (t - self.vars.t_prev)

            # Define integral and cap sum to a min/max
            self.vars.v_error_integral = \
                    np.fmax(np.fmin(self.vars.v_error_integral, 
                                    self.vars.integrator_max), 
                            self.vars.integrator_min)

            # Update throttle with PID to reach desired velocity
            throttle_output = self.vars.kp * self.vars.v_error + \
                              self.vars.ki * self.vars.v_error_integral + \
                              self.vars.kd * v_error_rate_of_change

            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            ######################################################
            ######################################################
            # LATERAL CONTROLLER
            

            # # Correction for heading error
            # yaw_path = np.arctan2(waypoints[-1][1] - y, waypoints[-1][0] - x)
            # heading_error = yaw_path - yaw  
                        
            
            # crosstrack_error = math.sqrt((closest_x - x)**2 + (closest_y - y)**2)

            # # Checks if the vehicle is to the right or left, adjusts the
            # crosstrack_error = abs(crosstrack_error) if crosstrack_error > 0 else -abs(crosstrack_error)

            # # Correction for crosstrack error
            # kg = 0.1 # gain k determined experimentally
            # ks = 0.1 # softening constant
            # crosstrack_steering = np.arctan(kg * crosstrack_error / ks + v)

            # # Steering Output Update    
            # steer_output = max(-1.22, min(1.22, abs(heading_error + crosstrack_steering)))


            # Find cross track error (assume point with closest distance)
            crosstrack_error = float("inf")
            crosstrack_vector = np.array([float("inf"), float("inf")])

            ce_idx = self.get_lookahead_index(self._lookahead_distance)
            crosstrack_vector = np.array([waypoints[ce_idx][0] - \
                                         x - self._lookahead_distance*np.cos(yaw), 
                                          waypoints[ce_idx][1] - \
                                         y - self._lookahead_distance*np.sin(yaw)])
            crosstrack_error = np.linalg.norm(crosstrack_vector)

            # set deadband to reduce oscillations
            # print(crosstrack_error)
            if crosstrack_error < self.vars.cross_track_deadband:
                crosstrack_error = 0.0

            # Compute the sign of the crosstrack error
            crosstrack_heading = np.arctan2(crosstrack_vector[1], 
                                            crosstrack_vector[0])
            crosstrack_heading_error = crosstrack_heading - yaw
            crosstrack_heading_error = \
                    (crosstrack_heading_error + self._pi) % \
                    self._2pi - self._pi

            crosstrack_sign = np.sign(crosstrack_heading_error)
    
            # Compute heading relative to trajectory (heading error)
            # First ensure that we are not at the last index. If we are,
            # flip back to the first index (loop the waypoints)
            if ce_idx < len(waypoints)-1:
                vect_wp0_to_wp1 = np.array(
                        [waypoints[ce_idx+1][0] - waypoints[ce_idx][0],
                         waypoints[ce_idx+1][1] - waypoints[ce_idx][1]])
                trajectory_heading = np.arctan2(vect_wp0_to_wp1[1], 
                                                vect_wp0_to_wp1[0])
            else:
                vect_wp0_to_wp1 = np.array(
                        [waypoints[0][0] - waypoints[-1][0],
                         waypoints[0][1] - waypoints[-1][1]])
                trajectory_heading = np.arctan2(vect_wp0_to_wp1[1], 
                                                vect_wp0_to_wp1[0])

            heading_error = trajectory_heading - yaw
            heading_error = \
                    (heading_error + self._pi) % self._2pi - self._pi

            # Update steering based on error
            steer_output = heading_error + \
                    np.arctan(self.vars.kp_heading * \
                              crosstrack_sign * \
                              crosstrack_error / \
                              (v + self.vars.k_speed_crosstrack))

            
            # SET CONTROLS OUTPUT
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

            
        # STORE VALUES FOR NEXT TIME STEP
        self.vars.x_prev = x
        self.vars.y_prev = y
        self.vars.v_prev = v
        self.vars.yaw_prev = yaw
        self.vars.t_prev = t
        self.vars.v_error_prev = self.vars.v_error
