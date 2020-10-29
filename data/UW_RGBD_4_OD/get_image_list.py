#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# '''
# Description: To get a list of absolute path for each images in ./images 
#              directory
# Usage: python3 get_image_list.py [list_filename]
# Author: BIgRunner
# Date: 20190827
# '''


import os
import sys

IMAGE_DIR = './images'

list_file = sys.argv[1]
with open(list_file, 'w') as f: 
  for image in os.listdir(IMAGE_DIR):
    f.write(os.path.abspath(os.path.join(IMAGE_DIR, image)))
    f.write('\n')
