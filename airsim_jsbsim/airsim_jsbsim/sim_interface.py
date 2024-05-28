from abc import ABC
import numpy as np
import time 
#garbage collection
import gc
from airsim_jsbsim.jsbsim_backend.simulator import FlightDynamics
from airsim_jsbsim.jsbsim_backend.aircraft import Aircraft, x8
from airsim_jsbsim.debug_utils import *
from simple_pid import PID
from airsim_jsbsim.guidance_control.autopilot import X8Autopilot
from airsim_jsbsim.guidance_control.navigation import WindEstimation
from airsim_jsbsim.conversions import feet_to_meters, meters_to_feet, ktas_to_mps, mps_to_ktas
from typing import Type, Tuple, Dict
from airsim_jsbsim.report_diagrams import ReportGraphs
from typing import Type, Tuple, Dict

"""
Adapter class to interface with the JSBSim API and other simulators
"""

class CLSimInterface():
    def __init__(self, 
                 init_conditions: dict,
                 aircraft: Aircraft = x8,
                 flight_dynamics_sim_hz: float = 240,
                 autopilot: X8Autopilot = None,
                 debug_level: int = 0,
                 use_airsim:bool = False,
                 ip_address:str='localhost',
                 airsim_frequency_hz:float=40.0):
        
        self.init_conditions = init_conditions
        self.aircraft = aircraft
        self.flight_dynamics_sim_hz = flight_dynamics_sim_hz
        self.debug_level = debug_level
        
        self.use_air_sim = use_airsim
        self.ip_address = ip_address
        self.airsim_frequency_hz = airsim_frequency_hz
        #this will initialize the simulator with the aircraft and initial conditions
        self.sim = FlightDynamics(aircraft=aircraft, 
                                  init_conditions=init_conditions, 
                                  debug_level=debug_level,
                                  sim_frequency_hz=flight_dynamics_sim_hz,
                                  use_airsim=use_airsim,
                                  ip_address=ip_address,
                                  airsim_frequency_hz=airsim_frequency_hz)
        self.sim.start_engines()
        self.sim.set_throttle_mixture_controls(0.3, 0)
        
        self.autopilot = autopilot
        if self.autopilot is None:
            self.autopilot = X8Autopilot(self.sim)
            
        self.wind = WindEstimation(self.sim)
        self.report = ReportGraphs(self.sim)
        self.over = False
        self.graph = DebugGraphs(self.sim)
        
    def run_backend(self, track_data:bool=True) -> None:
        self.sim.run()
        if track_data:
            self.graph.get_pos_data()
            self.graph.get_time_data()
            self.graph.get_angle_data()
            self.graph.get_airspeed()
    
    # def reset_backend(self) -> None:
    #     """
    #     Require a method to reset the simulator
    #     """
    #     raise NotImplementedError("Method reset_sim not implemented")
    
    def get_states(self) -> dict:
        return self.sim.get_states()

    def reset_backend(self, init_conditions:dict=None) -> None:
        """
        Reset the simulator to the initial conditions provided
        """
        if init_conditions is not None:
            # self.init_conditions = init_conditions
            # self.sim.reinitialise(init_conditions)
            if self.sim:
                aircraft = self.sim.aircraft
                debug_level = self.debug_level
                flight_dynamics_sim_hz = self.sim.sim_frequency_hz
                self.init_conditions = init_conditions
                self.sim.close()
                self.sim = None
                self.sim = FlightDynamics(aircraft=aircraft, 
                                    init_conditions=init_conditions, 
                                    debug_level=debug_level,
                                    sim_frequency_hz=flight_dynamics_sim_hz)
                self.autopilot = X8Autopilot(self.sim)
                self.graph = DebugGraphs(self.sim)
                # self.sim.reinitialise(init_conditions)
                # self.sim.initialise(dt, aircraft.jsbsim_id, init_conditions)
        else:
            self.sim.reinitialise(self.init_conditions)    

    
class PursuerInterface(CLSimInterface):
    """
    This class is an interface between the pursuer drone and the
    JSBSim flight dynamics. It is used to control the pursuer drone
    where the user can utilize a simple proportional navigation 
    or a pursuit guidance law to track the evader drone.
    """
    def __init__(self, 
                 init_conditions: dict,
                 evader_position: np.ndarray,
                 control_constraints: dict,
                 id_number: int,
                 nav_constant: float = 5,
                 aircraft: Aircraft = x8,
                 flight_dynamics_sim_hz: float = 100,
                 autopilot: X8Autopilot = None,
                 min_max_vels:np.ndarray = np.array([15, 30]),
                 debug_level: int = 0,
                 use_airsim:bool = False,
                 ip_address:str='localhost',
                 airsim_frequency_hz:float=40.0):
        
        super().__init__(init_conditions=init_conditions,
                         aircraft=aircraft,
                         flight_dynamics_sim_hz=flight_dynamics_sim_hz,
                         autopilot=autopilot,
                         debug_level=debug_level,
                         use_airsim=use_airsim,
                         ip_address=ip_address,
                         airsim_frequency_hz=airsim_frequency_hz)
        
        self.control_constraints = control_constraints
        self.nav_constant = nav_constant
        self.old_states = self.get_observation()
        self.min_max_vels = min_max_vels
        self.previous_heading_rad = self.old_states[5]
        self.old_evader_position = evader_position
        self.id = id_number

    def track_evader(self, evader_position: np.ndarray) -> None:
        current_states = self.get_observation()

    def vector_magnitude(self,v):
        """Calculate the magnitude of a vector."""
        return math.sqrt(v[0]**2 + v[1]**2)


    def cartesian_to_navigation_radians(self, 
            cartesian_angle_radians:float) -> float:
        """
        Converts a Cartesian angle in radians to a navigation 
        system angle in radians.
        North is 0 radians, East is π/2 radians, 
        South is π radians, and West is 3π/2 radians.
        
        Parameters:
        - cartesian_angle_radians: Angle in radians in the Cartesian coordinate system.
        
        Returns:
        - A navigation system angle in radians.
        """
        new_yaw = math.pi/2 - cartesian_angle_radians
        return new_yaw

    def pro_nav(self, target_states:np.ndarray) -> tuple:
        """
        Returns the control commands for the pursuer drone
        Keep in mind in the simulator the angle orientation is 
        defined as follows:
        
        North = 0 degrees
        East = 90 degrees
        South = 180 degrees
        West = 270 degrees
        
        If we are using cartesian coordinates we will have to map it 
        to the above orientation to get the correct angle 
        """
        current_states = self.get_observation()
        pursuer_vel_mag = current_states[-1]
        
        dx = target_states[0] - current_states[0]
        dy = target_states[1] - current_states[1]
        
        los = np.arctan2(dy, dx)
          
        target_vel = target_states[-1]
        target_vx = target_vel * np.cos(target_states[5])
        target_vy = target_vel * np.sin(target_states[5])
        
        dt = 1/self.flight_dynamics_sim_hz
        target_next_x = target_states[0] + (target_vx*dt)
        target_next_y = target_states[1] + (target_vy*dt)
        
        los_next = np.arctan2(target_next_y - current_states[1], 
                              target_next_x - current_states[0])
                
        pursuer_vx = pursuer_vel_mag * np.cos(current_states[5])
        pursuer_vy = pursuer_vel_mag * np.sin(current_states[5])
        
        los_rate = np.array([target_vx - pursuer_vx, 
                             target_vy - pursuer_vy])
        los_rate = np.arctan2(los_rate[1], los_rate[0])
        
        los_mag_vel = -np.linalg.norm(los_rate)
                
        los = self.cartesian_to_navigation_radians(los)

        los_rate = self.cartesian_to_navigation_radians(los_rate)
        
        los_rate = los_rate * dt
        
        return los, los_rate

    def pursuit_nav(self, target_states:np.ndarray) -> tuple:
        """
        Returns the control commands for the pursuer drone
        Keep in mind in the simulator the angle orientation is 
        defined as follows:
        
        North = 0 degrees
        East = 90 degrees
        South = 180 degrees
        West = 270 degrees
        
        If we are using cartesian coordinates we will have to map it 
        to the above orientation to get the correct angle 
        
        """
        current_states = self.get_observation()
        
        dx = target_states[0] - current_states[0]
        dy = target_states[1] - current_states[1]
        los = np.arctan2(dy, dx)        
                        
        los = self.cartesian_to_navigation_radians(los)
        yaw = current_states[5]
        error_los = abs(los - yaw)

        if error_los > np.deg2rad(20):
            vel_cmd = self.min_max_vels[0]
        else:
            vel_cmd = self.min_max_vels[1]
            
        vel_cmd = np.clip(vel_cmd, self.min_max_vels[0], 
                          self.min_max_vels[1])
        return los, vel_cmd
    
    def set_command(self, heading_cmd_rad:float,
                    v_cmd_ms:float, 
                    height_cmd_m:float,
                    control_hz:int=30) -> None:
        """
        Set the control commands from pro_nav to the aircraft
        """
        v_max = self.control_constraints['v_cmd_max']
        v_min = self.control_constraints['v_cmd_min']
        
        if v_cmd_ms >= v_max:
            v_cmd_ms = v_max
        elif v_cmd_ms <= v_min:
            v_cmd_ms = v_min
        
        if heading_cmd_rad > np.pi:
            heading_cmd_rad = heading_cmd_rad - 2*np.pi
        elif heading_cmd_rad < -np.pi:
            heading_cmd_rad = heading_cmd_rad + 2*np.pi
                    
        self.autopilot.heading_hold(np.rad2deg(heading_cmd_rad))    
        self.autopilot.altitude_hold(meters_to_feet(height_cmd_m))
        self.autopilot.airspeed_hold_w_throttle(mps_to_ktas(v_cmd_ms))
        self.run_backend()

    def reset_backend(self, init_conditions:dict=None) -> None:
        if init_conditions is not None:
            self.init_conditions = init_conditions
            self.sim.reinitialise(init_conditions)
            # self.sim = FlightDynamics(aircraft=self.aircraft, 
            #                           init_conditions=init_conditions)
        else:
            self.sim.reinitialise(self.init_conditions)

    def get_info(self) -> dict:
        """
        Might belong to parent class
        """
        return self.sim.get_states()

    def get_observation(self) -> np.ndarray:
        """
        Returns the observation space for the environment as 
        a numpy array, might belong to parent class
        """
        state_dict = self.sim.get_states()
        states = [
            state_dict['x'],
            state_dict['y'],
            state_dict['z'],
            state_dict['phi'],
            state_dict['theta'],
            state_dict['psi'],
            state_dict['airspeed'],
        ]

        return np.array(states, dtype=np.float32)
