import cv2
import numpy as np
import isect_segments_bentley_ottmann.poly_point_isect as bot
import math

img = cv2.imread('data/img2.jpg')
img = np.asarray(img)

img_height, img_width, _ = img.shape

def processImg(img):
  global img_height, img_width
  gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  blur = cv2.GaussianBlur(gray_img, (5,5),0)

  ret, bin_img = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY)

  # crop image
  CROP_HEIGHT = int(img_height/2)
  bin_img = bin_img[CROP_HEIGHT:,]

  edges = cv2.Canny(bin_img, 50, 150)

  rho = 1  # distance resolution in pixels of the Hough grid
  theta = np.pi / 180  # angular resolution in radians of the Hough grid
  threshold = 15  # minimum number of votes (intersections in Hough grid cell)
  min_line_length = 50  # minimum number of pixels making up a line
  max_line_gap = 20  # maximum gap in pixels between connectable line segments
  line_image = np.copy(img) * 0  # creating a blank to draw lines on

  # Run Hough on edge detected image
  # Output "lines" is an array containing endpoints of detected line segments
  lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                      min_line_length, max_line_gap)

  points = []
  for line in lines:
      for x1, y1, x2, y2 in line:
          points.append(((x1 + 0.0, y1 + 0.0), (x2 + 0.0, y2 + 0.0)))
          cv2.line(line_image, (x1, y1+CROP_HEIGHT), (x2, y2+CROP_HEIGHT), (255, 0, 0), 5)


  lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)
  # intersections = bot.isect_segments(points)
  # for inter in intersections:
  #   a, b = inter
  #   for i in range(3):
  #     for j in range(3):
  #       lines_edges[int(b) + i, int(a) + j] = [0, 255, 0]

  print(lines.shape)

  return lines_edges



processed_img = processImg(img)

# print(processed_img.shape)
cv2.imshow('Img', processed_img)
cv2.waitKey(2000)
cv2.destroyAllWindows()