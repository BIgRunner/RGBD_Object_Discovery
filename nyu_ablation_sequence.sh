#!/bin/bash
# Program:
#     This program is to extract planes from a sequence and compared 
#
# History:
# 20200911	BIgRunner	V0.0.1
# read -p "Please choose seeding method (0:Random, 1:Local, 2:Grid, 3:Grid Local): " seeding_method
# read -p "How many seeds to put: " seeds_count
# read -p "Please input size thresh in growing: " size_thresh
# read -p "Please input distance thresh in growing: " distance_thresh
# read -p "With boundary? (1 for yes, 0 for no): " boundary
# read -p "With normal contency check? (1 for yes, 0 for no): " norm_cont
# read -p "Please input normal difference in growing: " angle_diff
# read -p "With merge? (1 for yes, 0 for no): " merge
# read -p "Please input distance thresh in merging: " plane_distance_thresh
# read -p "Please input normal difference in merging: " plane_angle_diff

seeding_method=3
seeds_count=800
size_thresh=3000
boundary=0
angle_diff=0.4
plane_distance_thresh=0.1
plane_angle_diff=0.1

echo "" >> result/nyu_accuracy.txt
date >> result/nyu_accuracy.txt

for distance_thresh in 0.045 0.05 0.055 0.06
do
  for norm_cont in 0 1
  do
    for merge in 0 1
    do
  
      result_dir=./result/nyu_${seeding_method}_${seeds_count}_${size_thresh}_${distance_thresh}_${boundary}_${norm_cont}_${angle_diff}_${merge}_${plane_distance_thresh}_${plane_angle_diff}
      
      if [ ! -d result_dir ]; then
        mkdir -p ${result_dir}
      fi
      
      for file in ./data/NYU_DATASET/NYU_DEPTH/*
      do
        num=${file##*_}
        num=${num%.png}
        depth_file=./data/NYU_DATASET/NYU_DEPTH/nyu_${num}.png
        color_file=./data/NYU_DATASET/NYU_COLOR/nyu_${num}.png
        echo "# Depth file" > cfg/config_nyu.txt
        echo "depth=${depth_file}" >> cfg/config_nyu.txt
        echo "color=${color_file}" >> cfg/config_nyu.txt
        echo "save_path=${result_dir}/nyu_${num}_MS.png" >> cfg/config_nyu.txt
        echo "" >> cfg/config_nyu.txt
        echo "# camera" >> cfg/config_nyu.txt
        echo "camera.r0=196.444" >> cfg/config_nyu.txt
        echo "camera.c0=275.045" >> cfg/config_nyu.txt
        echo "camera.fr=582.691" >> cfg/config_nyu.txt
        echo "camera.fc=582.624" >> cfg/config_nyu.txt
        echo "camera.scale=1000.0" >> cfg/config_nyu.txt
        echo "" >> cfg/config_nyu.txt
        echo "# parameters for sementation" >> cfg/config_nyu.txt
        echo "seeding_method=${seeding_method}" >> cfg/config_nyu.txt
        echo "seeds_count=${seeds_count}" >> cfg/config_nyu.txt
        echo "size_thresh=${size_thresh}" >> cfg/config_nyu.txt
        echo "distance_thresh=${distance_thresh}" >> cfg/config_nyu.txt
        echo "" >> cfg/config_nyu.txt
        echo "# boundary" >> cfg/config_nyu.txt
        echo "boundary=${boundary}" >> cfg/config_nyu.txt
        echo "" >> cfg/config_nyu.txt
        echo "# normal concensus" >> cfg/config_nyu.txt
        echo "norm_cont=${norm_cont}" >> cfg/config_nyu.txt
        echo "angle_diff=${angle_diff}" >> cfg/config_nyu.txt
        echo "" >> cfg/config_nyu.txt
        echo "# for merge state" >> cfg/config_nyu.txt
        echo "merge=${merge}" >> cfg/config_nyu.txt
        echo "plane_distance_thresh=${plane_distance_thresh}" >> cfg/config_nyu.txt
        echo "plane_angle_diff=${plane_angle_diff}" >> cfg/config_nyu.txt
      	
        # segment
        ./bin/train_nyu
      done
      
      # >>> conda initialize >>>
      # !! Contents within this block are managed by 'conda init' !!
      __conda_setup="$('/home/runner/miniconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
      if [ $? -eq 0 ]; then
          eval "$__conda_setup"
      else
          if [ -f "/home/runner/miniconda3/etc/profile.d/conda.sh" ]; then
              . "/home/runner/miniconda3/etc/profile.d/conda.sh"
          else
              export PATH="/home/runner/miniconda3/bin:$PATH"
          fi
      fi
      unset __conda_setup
      # <<< conda initialize <<<
      conda activate cv4_env
  
      echo "| ${distance_thresh} | ${norm_cont} | ${merge} |" >> result/nyu_accuracy.txt
      python3 ./compare.py ./data/NYU_DATASET/NYU_GROUND_TRUTH ${result_dir} --all -t 0.5 -s >> result/nyu_accuracy.txt
      echo " " >> result/nyu_accuracy.txt
    done
  done
done