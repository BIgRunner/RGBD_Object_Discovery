import os
import cv2


with os.scandir("./select_rgb") as entries:
  for entry in entries:
    print(entry.name)
    raw_img = cv2.imread("./select_rgb/"+entry.name)
    new_img = raw_img[42:42+427,38:38+564]
    cv2.imwrite("./NYU_COLOR/"+entry.name, new_img)
