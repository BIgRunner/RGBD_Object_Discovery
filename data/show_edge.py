import cv2
import numpy as np

depth_image = cv2.imread("./NYU_DEPTH/nyu_2.png", -1)
color_image = cv2.imread("./NYU_COLOR/nyu_2.png")

color = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
c_x = cv2.Sobel(color, cv2.CV_32F, 1, 0)
c_y = cv2.Sobel(color, cv2.CV_32F, 0, 1)
c_abs_x = cv2.convertScaleAbs(c_x)
c_abs_y = cv2.convertScaleAbs(c_y)
c_delta = cv2.addWeighted(c_abs_x, 0.5, c_abs_y, 0.5, 0)
_, c_edge =cv2.threshold(c_delta, 30, 255, cv2.THRESH_BINARY)


d_lap = cv2.Laplacian(depth_image, cv2.CV_32F, ksize=5)
d_delta = cv2.absdiff(d_lap, np.zeros(d_lap.shape, np.float32))
_, d_edge = cv2.threshold(d_delta, 100.0, 255, cv2.THRESH_BINARY)

cv2.imshow("color", color_image)
cv2.imshow("depth", depth_image)
cv2.imshow("c_edge", c_edge)
cv2.imshow("d_edge", d_edge)
print(d_lap)
print(d_lap.max())
print(d_lap.min())
print(d_lap.dtype)
print(d_delta.max())
print(d_delta.min())
print(d_edge.dtype)
print(np.unique(d_edge))

cv2.waitKey()

