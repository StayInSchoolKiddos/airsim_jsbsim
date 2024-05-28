#  Airsim JSBSim
This repo is used to interface Airsim with ROS2, this is done by running JSBSim on the backend and using the MovePosition api from airsim to move the camera pawn, this mode requires you to be in Computer Vision mode.

## Install dependencies
- Make sure to install Airsim and Unreal Engine 
- If you are interfacing this with ROS with WSL2 clone this specific Airsim repo: https://github.com/StayInSchoolKiddos/AirSim and compile the code 

## Virtual environment
Before you start developing set up your virtual environment and install your Python dependencies
```
virtualenv venv #do this command once
source venv/bin/activate #activate your virtual environment
pip install -r requirements.txt
```
