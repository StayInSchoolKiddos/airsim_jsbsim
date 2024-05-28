#!/usr/bin/env python3

import airsim 
import numpy as np
import os 
import pprint
import time as time

from airsim_jsbsim.jsbsim_backend.aircraft import Aircraft, x8
from airsim_jsbsim.jsbsim_backend.simulator import FlightDynamics
from airsim_jsbsim.conversions import meters_to_feet, mps_to_ktas
from airsim_jsbsim.sim_interface import PursuerInterface

"""
This is a toy script to test the connection to AirSim to figure out how to move the camera
https://microsoft.github.io/AirSim/image_apis/

The Unreal Engine Coordinate System is left-handed, 
with the X-axis pointing forward, 
the Y-axis pointing to the right, 
and the Z-axis pointing down.

The NED coordinate system is right-handed,
with the X-axis pointing north,
the Y-axis pointing east,
and the Z-axis pointing down.

What should be happening is the following:
-x jsbsim is running in the background
-x airsim is running in the foreground
-x the vehicle is moving in airsim
-x I need to get the position of the vehicle in jsbsim 
- Then convert it from NED to Unreal Engine Coordinate System
- Then move the camera to that position using the airsim api

TO DOS:
- Log the data and see what conversions I need to make
- Synch the time between the JSBSim and AirSim
"""


pp = pprint.PrettyPrinter(indent=4)

#get WSL_HOST_IP
WSL_HOST_IP = os.getenv('WSL_HOST_IP', 'localhost')
print('WSL_HOST_IP: ', type(WSL_HOST_IP))

aircraft = x8
init_state_dict = {
    "ic/u-fps": mps_to_ktas(25),
    "ic/v-fps": 0.0,
    "ic/w-fps": 0.0,
    "ic/p-rad_sec": 0.0,
    "ic/q-rad_sec": 0.0,
    "ic/r-rad_sec": 0.0,
    "ic/h-sl-ft": meters_to_feet(50),
    "ic/long-gc-deg": 0.0,
    "ic/lat-gc-deg": 0.0,
    "ic/psi-true-deg": 20.0,
    "ic/theta-deg": 0.0,
    "ic/phi-deg": 0.0,
    "ic/alpha-deg": 0.0,
    "ic/beta-deg": 0.0,
    "ic/num_engines": 1,
}

control_constraints = {
    'u_phi_min':  -np.deg2rad(45),
    'u_phi_max':   np.deg2rad(45),
    'u_theta_min':-np.deg2rad(10),
    'u_theta_max': np.deg2rad(10),
    'u_psi_min':  -np.deg2rad(45),
    'u_psi_max':   np.deg2rad(45),
    'v_cmd_min':   15,
    'v_cmd_max':   30
}

# fdm = FlightDynamics(sim_frequency_hz=250,
#                       aircraft=aircraft,
#                       init_conditions=init_state_dict,
#                       use_airsim=True,
#                       ip_address=WSL_HOST_IP)
evader_position = [500, 500, 30]
evader_position = np.array(evader_position)
cl_interface = PursuerInterface(
    flight_dynamics_sim_hz=250,
    aircraft=aircraft,
    init_conditions=init_state_dict,
    use_airsim=True,
    ip_address=WSL_HOST_IP,
    control_constraints=control_constraints,
    evader_position=evader_position,
    id_number=1
)

fdm = cl_interface.sim
# relative_update = fdm.airsim_frequency_hz / fdm.sim_frequency_hz

while True:
    
    #there's gotta be a better way to do this
    time.sleep(1/250)
    
    los, vel_cmd = cl_interface.pursuit_nav(evader_position)
    cl_interface.set_command(los, vel_cmd, 0.5)
    cl_interface.run_backend()
    
    #get the states of the vehicle
    observation = cl_interface.get_observation()
    print("Observation: ", observation)
    
    #wit this position need to convert it to the Unreal Engine Coordinate System
    #then we need to move the camera to that position
    fdm.update_airsim()
    #the api to get the pose of the vehicle
    pose = fdm.client.simGetVehiclePose()
    print("Pose: ", pose)
    print("\n")
    
    
