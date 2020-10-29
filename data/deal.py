import os
import re
import cv2


with os.scandir("./NYU_DEPTH_SHOW") as entries:
  for entry in entries:
    ind= int(re.search(r"(\d+)", entry.name).group(1))
    os.popen("cp ./images/" +"{:05d}".format(ind)  +"_color.png ./select_rgb/nyu_"+str(ind)+".png")
