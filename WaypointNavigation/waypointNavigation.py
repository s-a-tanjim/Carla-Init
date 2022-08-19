'''
https://github.com/carla-simulator/carla/issues/3883
'''

import time
from carlaEnv import CarlaEnv


if __name__=='__main__':
  try:
    env = CarlaEnv(debug=True)

    points = env.get_two_spawn_points(random_choose=True)
    
    env.traverse_between_points(points[0],points[1])
    # print(points[0])

  except KeyboardInterrupt:
    print("End!")
