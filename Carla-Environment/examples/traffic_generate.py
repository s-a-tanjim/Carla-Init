import random
import carla


class TrafficGenerator():
  def __init__(self, client, world, vehicle_count = 0, walker_count = 0):
    self.client = client
    self.world = world
    self.vehicle_count = vehicle_count
    self.walker_count = walker_count

    self.blueprint_lib = world.get_blueprint_library()
    self.spawn_points = self.world.get_map().get_spawn_points()
    self.vehicles = []
    self.walkers_actor = []


  def generate(self):
    if self.vehicle_count > len(self.spawn_points):
      print("Couldn't generate traffic. Available spawn point is greater than vehicle count!")
      return
    
    for i in range(self.vehicle_count):
      vehicle_bp = random.choice(self.blueprint_lib.filter('vehicle')) 
      npc = self.world.try_spawn_actor(vehicle_bp, random.choice(self.spawn_points))
      if(npc):
        npc.set_autopilot(True)
        self.vehicles.append(npc)

    print("Generated random traffic")

    if self.walker_count > 0:
      self.spawn_walker()
      print("Generated walker")


  def spawn_walker(self):
    blueprintsWalkers = self.blueprint_lib.filter("walker.*")

    # 1. Take all the random locations to spawn
    spawn_points = []
    for i in range(self.walker_count):
      spawn_point = carla.Transform()
      spawn_point.location = self.world.get_random_location_from_navigation()
      if (spawn_point.location != None):
        spawn_points.append(spawn_point)

    # 2. Build the batch of commands to spawn the pedestrians
    batch = []
    for spawn_point in spawn_points:
      walker_bp = random.choice(blueprintsWalkers)
      batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

    # 2.1 apply the batch
    walkers_list = []
    results = self.client.apply_batch_sync(batch, True)
    for i in range(len(results)):
      if results[i].error:
        print(results[i].error)
      else:
        walkers_list.append({"id": results[i].actor_id})

    # 3. Spawn walker AI controllers for each walker
    batch = []
    walker_controller_bp = self.blueprint_lib.find('controller.ai.walker')
    for i in range(len(walkers_list)):
      batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

    # 3.1 apply the batch
    results = self.client.apply_batch_sync(batch, True)
    for i in range(len(results)):
      if results[i].error:
        print(results[i].error)
      else:
        walkers_list[i]["con"] = results[i].actor_id

    all_id = []
    # 4. Put altogether the walker and controller ids
    for i in range(len(walkers_list)):
      all_id.append(walkers_list[i]["con"])
      all_id.append(walkers_list[i]["id"])
    self.walkers_actor = self.world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    self.world.wait_for_tick()

    # 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
    for i in range(0, len(self.walkers_actor), 2):
      # start walker
      self.walkers_actor[i].start()
      # set walk to random point
      self.walkers_actor[i].go_to_location(self.world.get_random_location_from_navigation())
      # random max speed
      self.walkers_actor[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)




  def destroy(self):
    self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles])

    for i in range(0, len(self.walkers_actor), 2):
      # stop walker controller
      self.walkers_actor[i].stop()

    self.client.apply_batch([carla.command.DestroyActor(x) for x in self.walkers_actor])
    print("Destroyed traffic actor")
  
  def __del__(self):
    self.destroy()


def main():
  client = carla.Client('localhost', 2000)
  client.set_timeout(10.0)
  world = client.get_world()

  weather = carla.WeatherParameters(
    cloudiness=99.0,
    precipitation=30.0,
    sun_altitude_angle=80.0,
    rayleigh_scattering_scale=0,
  )
  world.set_weather(weather)

  traffic_gen = TrafficGenerator(client=client, world=world, vehicle_count=50, walker_count=50)

  traffic_gen.generate()

  try:
    while True:
      pass
  except KeyboardInterrupt:
    return


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("End")