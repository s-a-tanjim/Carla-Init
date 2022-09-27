import carla
import random
import time
import math
import matplotlib.pyplot as plt
import numpy as np

from agents.navigation.controller import VehiclePIDController
from agents.navigation.global_route_planner import GlobalRoutePlanner


class CarlaEnv:

  WAYPOINT_COLOR = carla.Color(r=3, g=211, b=252, a=255)
  PLOT_PID_GRAPH = True

  def __init__(self, debug = False):
    # Connect to the client and get the world object
    self.client = carla.Client('localhost', 2000)
    self.client.set_timeout(10)
    # print(self.client.get_available_maps())
    self.world = self.client.get_world() # Access all objects
    # self.world = self.client.load_world('Town04') # Load specific map
    self.map = self.world.get_map()
  
    # Get the blueprint library and the spawn points for the map
    self.blueprint_lib = self.world.get_blueprint_library() # access all the blueprint
    self.spawn_points = self.map.get_spawn_points()

    # Setting up weather
    weather = carla.WeatherParameters(
        cloudiness=99.0,
        precipitation=30.0,
        sun_altitude_angle=80.0,
        rayleigh_scattering_scale=0,
    )
    self.world.set_weather(weather)

    # Generate a collection of waypoints
    # waypoint_list = self.map.generate_waypoints(2.0)

    # Generate road topology
    # self.waypoint_tuple_list = self.map.get_topology()

    self.vehicle_blueprint = None
    self.vehicle = None
    self.debug = debug
    self.traffic_vehicles = []
    self.sensors_actor = []

    self.target_speed_plot = []
    self.current_speed_plot = []
    self.target_plot = []
    self.throttle_plot = []
    self.steer_plot = []
    

  # def get_topology_waypoint(self):
  #   path = random.choice(self.map.get_topology())
  #   start_point = carla.Location(path[0].transform.location)
  #   end_point = carla.Location(path[1].transform.location)
  #   return (start_point, end_point)

  def generate_random_traffic(self):
    # Add traffic to the simulation
    for i in range(30): 
      vehicle_bp = random.choice(self.blueprint_lib.filter('vehicle')) 
      npc = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_points))
      if(npc):
        npc.set_autopilot(True)
        self.traffic_vehicles.append(npc)
  

  def get_two_spawn_points(self, random_choose = True):
    if random_choose:
      while True:
        start_point = random.choice(self.spawn_points)
        end_point = random.choice(self.spawn_points)
        if(start_point!=end_point):
          break
    else:
      start_point = self.spawn_points[0]
      end_point = self.spawn_points[len(self.spawn_points)-1]
    return (start_point, end_point)


  def spawn_vehicle(self, spawn_point = None):
    # Get the blueprint for the vehicle you want
    self.vehicle_blueprint = self.blueprint_lib.find('vehicle.lincoln.mkz_2020')
    # Try spawning the vehicle at a randomly chosen spawn point
    if(spawn_point==None):
      spawn_point = self.spawn_points[0]

    self.vehicle = self.world.try_spawn_actor(self.vehicle_blueprint, spawn_point)

    # Move the spectator behind the vehicle 
    spectator = self.world.get_spectator() 
    transform = carla.Transform(self.vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),self.vehicle.get_transform().rotation) 
    spectator.set_transform(transform) 

    # Spawn an RGB cammera with an offset from the vehicle center
    camera_bp = self.blueprint_lib.find('sensor.camera.rgb') 
    camera_init_trans = carla.Transform(carla.Location(x=2.5, z=0.7)) #Change this to move camera
    camera_bp.set_attribute("fov","110")
    camera = self.world.spawn_actor(camera_bp, camera_init_trans, attach_to=self.vehicle)
    self.sensors_actor.append(camera)
    camera.listen(lambda image: self.rgb_camera_callback(image))

    # self.vehicle.set_autopilot(True)


  def rgb_camera_callback(self, img):
    pass

  def traverse_from_topology(self, start_waypoint, end_waypoint):
    # self.spawn_vehicle(start_waypoint.transform)
    self.spawn_vehicle()

    while True:
      car_waypoint = self.map.get_waypoint(self.vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving | carla.LaneType.Sidewalk))
      if(car_waypoint == end_waypoint):
        break
      next_waypoint = random.choice(car_waypoint.next(2.0))
      print(car_waypoint)
      print(next_waypoint)
      print("------------------------------------")

      change_yaw = car_waypoint.transform.rotation.yaw - next_waypoint.transform.rotation.yaw
      if(change_yaw>0):
        change_yaw = change_yaw/next_waypoint.transform.rotation.yaw
      elif(change_yaw<0):
        change_yaw = change_yaw/car_waypoint.transform.rotation.yaw
      change_yaw = change_yaw/2

      # self.vehicle.apply_control(carla.VehicleControl(throttle = 1.0, steer = change_yaw))
      self.vehicle.set_transform(next_waypoint.transform)
      time.sleep(0.5)
  

  def traverse_between_points(self, start_point, end_point):
    if self.vehicle==None:
      self.spawn_vehicle(start_point)
    
    self.generate_random_traffic()

    #  start_point = carla.Location(self.spawn_points[0].location)
    # end_point = carla.Location(self.spawn_points[len(self.spawn_points)-1].location)
    # print(type(start_point))
    start_point = carla.Location(start_point.location)
    end_point = carla.Location(end_point.location)

    grp = GlobalRoutePlanner(self.map, sampling_resolution=2)
    w1 = grp.trace_route(start_point, end_point) 
    
    if(self.debug):
      self.world.debug.draw_point(start_point ,color=self.WAYPOINT_COLOR ,size=0.6 ,life_time=120.0)
      self.world.debug.draw_point(end_point ,color=self.WAYPOINT_COLOR ,size=0.6 ,life_time=120.0)

    wps=[]
    for i in range(len(w1)):
      wps.append(w1[i][0])
      if(self.debug):
        self.world.debug.draw_point(w1[i][0].transform.location ,color=self.WAYPOINT_COLOR ,size=0.2 ,life_time=120.0)

    PID = self.setup_PID()

    speed = 40
    self.drive_through_plan(wps, speed, PID)


  def drive_through_plan(self, planned_route, speed, PID):
    """
      This function drives throught the planned_route with the speed passed in the argument
    """
    try:
      i=0
      target=planned_route[0]
      # print("target:")
      # print(type(target))
      # print(target)
      if self.PLOT_PID_GRAPH:
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plotter, = plt.plot(self.current_speed_plot, color='g')
        plotter1, = plt.plot(self.target_speed_plot, color='r')
        # giving a title to my graph
        plt.title('Current Speed (KMh)!')
      
      iteration = 0
      while True:
        vehicle_loc= self.vehicle.get_location()
        distance_v = self.find_dist_veh(vehicle_loc,target)
        control = PID.run_step(speed, target)
        # print(type(control))
        # print(control)
        # print("-------------------")
        self.vehicle.apply_control(control)
        
        if i==(len(planned_route)-1):
          print("last waypoint reached")
          break 
        
        if (distance_v<3.5):
          control = PID.run_step(speed, target)
          self.vehicle.apply_control(control)
          i=i+1
          target=planned_route[i]

        if self.PLOT_PID_GRAPH:
          vel = self.vehicle.get_velocity()
          kmp = (3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)) # kilometer per hour

          print(kmp)

          self.current_speed_plot.append(kmp)
          self.target_speed_plot.append(speed)
          ax.plot(self.current_speed_plot, color='g')
          ax.plot(self.target_speed_plot, color='r')
          fig.canvas.draw()
          fig.canvas.flush_events()

          iteration += 1
          
      control = PID.run_step(0,planned_route[len(planned_route)-1])
      self.vehicle.apply_control(control)
    except KeyboardInterrupt:
      print("KeyboardInterrupt")
      

  def setup_PID(self):
    """
      This function creates a PID controller for the vehicle passed to it 
    """
    args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07
            ,'dt': 1.0 / 10.0
            }

    args_long_dict = {
            'K_P': 1,
            'K_D': 0.0,
            'K_I': 0.75
            ,'dt': 1.0 / 10.0
            }

    PID = VehiclePIDController(self.vehicle ,args_lateral=args_lateral_dict ,args_longitudinal=args_long_dict)
    return PID


  def find_dist_veh(self, vehicle_loc,target):
    dist = math.sqrt( (target.transform.location.x - vehicle_loc.x)**2 + (target.transform.location.y - vehicle_loc.y)**2 )
    return dist


  def destroy_actor(self):
    if self.vehicle:
      self.vehicle.destroy()

    self.client.apply_batch([carla.command.DestroyActor(x) for x in self.traffic_vehicles])

    self.client.apply_batch([carla.command.DestroyActor(x) for x in self.sensors_actor])


  def __del__(self):
    self.destroy_actor()
