'''
Basic control of carla vehicle using keyboard

NB: Run in superuser mode (for keyboard module)

Commands:
w - > Forward
s - > backward
a - > left
d - > right
q - > end the simulator
other - > stop


'''

import carla
import random
import time
import cv2
import numpy as np
import keyboard


class CarlaEnv:

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

    self.vehicle_blueprint = None
    self.vehicle = None
    self.debug = debug
    self.traffic_vehicles = []
    self.sensors_actor = []
    self.camera_data = None


  def generate_random_traffic(self):
    # Add traffic to the simulation
    for i in range(30): 
      vehicle_bp = random.choice(self.blueprint_lib.filter('vehicle')) 
      npc = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_points))
      if(npc):
        npc.set_autopilot(True)
        self.traffic_vehicles.append(npc)
  

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
    
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    self.camera_data = {'image': np.zeros((image_h, image_w, 4))}

    camera.listen(lambda image: self.rgb_camera_callback(image, self.camera_data))
    # self.vehicle.set_autopilot(True)


  def rgb_camera_callback(self, img, data_dict):
    data_dict['image'] = np.reshape(np.copy(img.raw_data), (img.height, img.width, 4))


  def drive_left(self):
    self.vehicle.apply_control(carla.VehicleControl(throttle = 1, steer = -1))

  def drive_right(self):
    self.vehicle.apply_control(carla.VehicleControl(throttle = 1, steer = 1))

  def drive_forward(self):
    self.vehicle.apply_control(carla.VehicleControl(throttle = 1, steer = 0))

  def drive_backward(self):
    self.vehicle.apply_control(carla.VehicleControl(throttle = -1, steer = 0))

  def drive_stop(self):
    self.vehicle.apply_control(carla.VehicleControl(throttle = 0, steer = 0))


  def drive_vehicle(self, dir):
    if(dir=="w"):
      self.drive_forward()
    elif(dir=='s'):
      self.drive_backward()
    elif(dir=='a'):
      self.drive_left()
    elif(dir=='d'):
      self.drive_right()
    else:
      self.drive_stop()


  def destroy_actor(self):
    if self.vehicle:
      self.vehicle.destroy()

    self.client.apply_batch([carla.command.DestroyActor(x) for x in self.traffic_vehicles])

    self.client.apply_batch([carla.command.DestroyActor(x) for x in self.sensors_actor])


  def __del__(self):
    self.destroy_actor()


if __name__ == '__main__':
  cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)

  car = CarlaEnv()
  car.generate_random_traffic()
  car.spawn_vehicle()

  try:
    while True:
      
      event = keyboard.read_event()
      if event.event_type == keyboard.KEY_DOWN:
          key = event.name
          car.drive_vehicle(key)
          print(f'Pressed: {key}')
          if key == 'q':
              break
      
      cv2.imshow('RGB Camera',car.camera_data['image'])
      cv2.waitKey(1)

  except KeyboardInterrupt:
    cv2.destroyAllWindows()
    print("Exit")
    