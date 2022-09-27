import carla
import math
import random
import time
import numpy as np
import cv2

# Connect to the client and get the world object
client = carla.Client('localhost', 2000) 
world = client.get_world() # Access all objects
map = world.get_map()

# Get the blueprint library and the spawn points for the map
bp_lib = world.get_blueprint_library() # access all the blueprint
spawn_points = map.get_spawn_points() 

# Setting up weather
weather = carla.WeatherParameters(
    cloudiness=99.0,
    precipitation=30.0,
    sun_altitude_angle=80.0,
    rayleigh_scattering_scale=0,
)

world.set_weather(weather)

# Setting up vehicles
# Get the blueprint for the vehicle you want
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 

# Try spawning the vehicle at a randomly chosen spawn point
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[0])

# Move the spectator behind the vehicle 
spectator = world.get_spectator() 
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
spectator.set_transform(transform) 

# Plot Map Image
map_image = np.zeros((512,512,3), dtype=np.uint8)
MAP_COLOR = (241,241,241)
cv2.namedWindow("Map", cv2.WINDOW_NORMAL)

# <class 'carla.libcarla.Waypoint'>
# Waypoint(Transform(Location(x=-72.781799, y=24.481796, z=0.000000), Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)))


vehicle.set_autopilot(True)

max_x_val = 0
min_x_val = 0
max_y_val = 0
min_y_val = 0

while True:
  try:
    # Nearest waypoint in the center of a Driving or Sidewalk lane.
    waypoint01 = map.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
    
    print(waypoint01.transform.location.x)

    if(waypoint01.transform.location.x > max_x_val):
      max_x_val = waypoint01.transform.location.x
    if(waypoint01.transform.location.x < min_x_val):
      min_x_val = waypoint01.transform.location.x

    if(waypoint01.transform.location.y > max_y_val):
      max_y_val = waypoint01.transform.location.y
    if(waypoint01.transform.location.y < min_y_val):
      min_y_val = waypoint01.transform.location.y

    map_image[int(waypoint01.transform.location.x)+256,int(waypoint01.transform.location.y)+256] = MAP_COLOR

    cv2.imshow("Map",map_image)
    cv2.waitKey(1)

    # print(type(waypoint01))
    print(waypoint01)
  
  except KeyboardInterrupt:
    print(f"max_x_val: {max_x_val}")
    print(f"min_x_val: {min_x_val}")
    print(f"max_y_val: {max_y_val}")
    print(f"min_y_val: {min_y_val}")
    break

vehicle.destroy()