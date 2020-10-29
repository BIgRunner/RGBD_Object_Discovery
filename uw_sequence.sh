#!/bin/bash
# Program:
#     This program is to extract planes from a sequence and compared 
#
# History:
# 20200911	BIgRunner	V0.0.1
read -p "Please choose seeding method (0:Random, 1:Local, 2:Grid, 3:Grid Local): " seeding_method
read -p "How many seeds to put: " seeds_count
read -p "Please input size thresh in growing: " size_thresh
read -p "Please input distance thresh in growing: " distance_thresh
read -p "With boundary? (1 for yes, 0 for no): " boundary
read -p "With normal contency check? (1 for yes, 0 for no): " norm_cont
read -p "Please input normal difference in growing: " angle_diff
read -p "With merge? (1 for yes, 0 for no): " merge
read -p "Please input distance thresh in merging: " plane_distance_thresh
read -p "Please input normal difference in merging: " plane_angle_diff

result_dir=./data/UW_RGBD_4_OD/visual

if [ ! -d result_dir ]; then
  mkdir -p ${result_dir}
fi

for file in ./data/UW_RGBD_4_OD/depths/*
do
  num=${file##*/}
  num=${num%-depth.png}
  visual_dir=${result_dir}/${num}
  if [ ! -d visual_dir ]; then
    mkdir ${visual_dir}
  fi
  depth_file=./data/UW_RGBD_4_OD/depths/${num}-depth.png
  color_file=./data/UW_RGBD_4_OD/images/${num}-color.png
  saliency_file=./data/UW_RGBD_4_OD/saliency/${num}-color_saliency.png
  echo "# Depth file" > cfg/config_uw.txt
  echo "depth=${depth_file}" >> cfg/config_uw.txt
  echo "color=${color_file}" >> cfg/config_uw.txt
  echo "saliency=${saliency_file}" >> cfg/config_uw.txt
  echo "save_path=${visual_dir}" >> cfg/config_uw.txt
  echo "" >> cfg/config_uw.txt
  echo "# camera" >> cfg/config_uw.txt
  echo "camera.r0=240.0" >> cfg/config_uw.txt
  echo "camera.c0=320.0" >> cfg/config_uw.txt
  echo "camera.fr=518.0" >> cfg/config_uw.txt
  echo "camera.fc=519.0" >> cfg/config_uw.txt
  echo "camera.scale=5000.0" >> cfg/config_uw.txt
  echo "" >> cfg/config_uw.txt
  echo "# parameters for sementation" >> cfg/config_uw.txt
  echo "seeding_method=${seeding_method}" >> cfg/config_uw.txt
  echo "seeds_count=${seeds_count}" >> cfg/config_uw.txt
  echo "size_thresh=${size_thresh}" >> cfg/config_uw.txt
  echo "distance_thresh=${distance_thresh}" >> cfg/config_uw.txt
  echo "" >> cfg/config_uw.txt
  echo "# boundary" >> cfg/config_uw.txt
  echo "boundary=${boundary}" >> cfg/config_uw.txt
  echo "" >> cfg/config_uw.txt
  echo "# normal concensus" >> cfg/config_uw.txt
  echo "norm_cont=${norm_cont}" >> cfg/config_uw.txt
  echo "angle_diff=${angle_diff}" >> cfg/config_uw.txt
  echo "" >> cfg/config_uw.txt
  echo "# for merge state" >> cfg/config_uw.txt
  echo "merge=${merge}" >> cfg/config_uw.txt
  echo "plane_distance_thresh=${plane_distance_thresh}" >> cfg/config_uw.txt
  echo "plane_angle_diff=${plane_angle_diff}" >> cfg/config_uw.txt
	
  echo ${depth_file}
  # segment
  ./bin/train_uw
done

# # >>> conda initialize >>>
# # !! Contents within this block are managed by 'conda init' !!
# __conda_setup="$('/home/runner/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
# if [ $? -eq 0 ]; then
#     eval "$__conda_setup"
# else
#     if [ -f "/home/runner/miniconda3/etc/profile.d/conda.sh" ]; then
#         . "/home/runner/miniconda3/etc/profile.d/conda.sh"
#     else
#         export PATH="/home/runner/miniconda3/bin:$PATH"
#     fi
# fi
# unset __conda_setup
# # <<< conda initialize <<<
# conda activate cv4_env
# python3 ./compare.py ./data/NYU_GROUND_TRUTH ${result_dir} --all -t 0.5 -s 
