'''
https://hackthedeveloper.com/lane-detection-opencv-python/
https://machinelearningknowledge.ai/image-segmentation-in-python-opencv/

'''


from cgitb import grey
import carla
import math
import random
import time
import numpy as np
import cv2

IMG_WIDTH = 800
IMG_HEIGHT = 600

LOWER_THRESHOLD = 50
UPPER_THRESHOLD = 150

# Connect to the client and get the world object
client = carla.Client('localhost', 2000) 
world = client.get_world() # Access all objects

# Get the blueprint library and the spawn points for the map
bp_lib = world.get_blueprint_library() # access all the blueprint
spawn_points = world.get_map().get_spawn_points() 

weather = carla.WeatherParameters(
    cloudiness=99.0,
    precipitation=30.0,
    sun_altitude_angle=80.0,
    rayleigh_scattering_scale=0,
)
world.set_weather(weather)

# Get the blueprint for the vehicle you want
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 

# Try spawning the vehicle at a randomly chosen spawn point
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Move the spectator behind the vehicle 
spectator = world.get_spectator() 
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
spectator.set_transform(transform)

# Spawn an RGB cammera with an offset from the vehicle center
camera_init_trans = carla.Transform(carla.Location(x=2.5, z=0.7))
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute("image_size_x", f"{IMG_WIDTH}")
camera_bp.set_attribute("image_size_y", f"{IMG_HEIGHT}")
camera_bp.set_attribute("fov","110")
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

def region_of_interest(img):
  height, width = img.shape
  polygon = np.array([[0, height], [220, 460], [660, 400], [width, height]])
  mask = np.zeros_like(img)
  
  cv2.fillPoly(mask, polygon, 255)
  masked_image = cv2.bitwise_and(img, mask)
  return masked_image

def draw_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)

    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    return img

def lanesDetection(img):
  # print(img.shape)
  height, width = img.shape

  region_of_interest_vertices = [
      (10, height), (width/2, height/1.37), (width-30, height)
  ]
  gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
  blur = cv2.GaussianBlur(gray_img, (5,5),0)
  edge = cv2.Canny(blur, 100, 200, apertureSize=3)
  
  cropped_image = region_of_interest(
      edge, np.array([region_of_interest_vertices], np.int32))
  return cropped_image
  lines = cv2.HoughLinesP(cropped_image, rho=2, theta=np.pi/180,
                          threshold=50, lines=np.array([]), minLineLength=10, maxLineGap=30)
  print(lines)  
  image_with_lines = draw_lines(img, lines)
  # plt.imshow(image_with_lines)
  # plt.show()
  return image_with_lines


# RGB Camera
def rgb_camera_callback(image,data_dict):
  img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
  # img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

  # res = lanesDetection(img)

  data_dict['rgb_image'] = img
  print(data_dict['rgb_image'].shape)


# Show Stream RGB Camera Feedback
camera_data = {'rgb_image': np.zeros((IMG_HEIGHT, IMG_WIDTH, 4))}
camera.listen(lambda image: rgb_camera_callback(image, camera_data))

cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
cv2.waitKey(2000)
flag = True
while True:
  try:
    cv2.imshow('RGB Camera', camera_data['rgb_image'])
    cv2.waitKey(1)
    if flag:
      flag = False
      cv2.imwrite('img3.jpg', camera_data['rgb_image'])
  except KeyboardInterrupt:
    vehicle.destroy()
    cv2.destroyAllWindows()
    break

