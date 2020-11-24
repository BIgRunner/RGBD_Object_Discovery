#!/usr/bin/env python3
# -*- coding: utf-8 -*-

################################################
# Author: Runner
# Description: Colorize nyu results for visual comparison
################################################

import cv2
import os
import numpy as np

common_color = [(100, 100, 100), (80, 127, 255), (35, 142, 107), (158, 168, 3), (31, 102, 156), (214, 112, 218), (255,0,0), (202, 230, 252), (20, 97, 199), (140, 199, 0), (71, 99, 255), (255, 255, 0), (203, 192, 255), (250, 51, 153), (18, 153, 255), (87, 207, 227), (0, 215, 255), (64, 125, 255), (132, 227, 255), (0, 128, 255), (33, 145, 237), (0, 102, 85), (112, 25, 25), (34, 139, 34), (205, 224, 64), (140, 180, 210), (42, 42, 128), (143, 143, 188), (45, 82, 160)]

MS_DIR = "result/nyu_3_1600_3000_0.03_0_1_0.4_1_0.1_0.1"
VISUAL_DIR = "visual/nyu_visual"

for f in os.listdir(MS_DIR):
  file_name = os.path.join(MS_DIR,f)
  if not file_name.endswith(".png"):
    continue
  ms = cv2.imread(file_name,-1)
  color = np.zeros((ms.shape[0], ms.shape[1], 3), dtype=np.uint8)
  for r in range(ms.shape[0]):
    for c in range(ms.shape[1]):
      idx = ms[r, c]
      color[r,c] = common_color[idx]
  cv2.imwrite(os.path.join(VISUAL_DIR, f), color)

