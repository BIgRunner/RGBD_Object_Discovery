#!/bin/bash
# Program:
#     This program is to extract planes from a sequence and compared 
#
# History:
# 20200911	BIgRunner	V0.0.1
# read -p "Please input experiment state: " exper_state
# read -p "Sequence start index: " start
# read -p "Sequence end index: " end
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

exper_state=train
start=0
end=10
size_thresh=100
distance_thresh=0.002
boundary=0
angle_diff=0.33
merge=1
plane_distance_thresh=0.005
plane_angle_diff=0.05


echo "" >> result/abw_train_results.txt
date >> result/abw_train_results.txt
for seeding_method in 3
do
  for seeds_count in 150 300 600 900 1200 1500 1800 2100 2400 2700 3000
  do
    for norm_cont in 1
    do  
      exper_state=$(echo ${exper_state} | tr -s '[:upper:]' '[:lower:]')
      result_dir=./result/abw_unsort_${seeding_method}_${seeds_count}_${size_thresh}_${distance_thresh}_${boundary}_${norm_cont}_${angle_diff}_${merge}_${plane_distance_thresh}_${plane_angle_diff}
      if [ ! -d result_dir ]; then
        mkdir -p ${result_dir}
      fi

      for num in $(seq ${start} $((${end}-1)))
      do
        depth_file=./data/ABW_DATASET/abw_depth/abw_${exper_state}_${num}.png
        color_file=./data/ABW_DATASET/abw_color/abw_${exper_state}_${num}.png
        echo "# Depth file" > cfg/config_abw.txt
        echo "depth=${depth_file}" >> cfg/config_abw.txt
        echo "color=${color_file}" >> cfg/config_abw.txt
        echo "save_path=${result_dir}/abw_${exper_state}_${num}_ms-seg.png" >> cfg/config_abw.txt
        echo "" >> cfg/config_abw.txt
        echo "# camera" >> cfg/config_abw.txt
        echo "camera.r0=255.0" >> cfg/config_abw.txt
        echo "camera.c0=255.0" >> cfg/config_abw.txt
        echo "camera.fr=2337.212" >> cfg/config_abw.txt
        echo "camera.fc=1610.982" >> cfg/config_abw.txt
        echo "camera.scale=773.545" >> cfg/config_abw.txt
        echo "" >> cfg/config_abw.txt
        echo "# parameters for sementation" >> cfg/config_abw.txt
        echo "seeding_method=${seeding_method}" >> cfg/config_abw.txt
        echo "seeds_count=${seeds_count}" >> cfg/config_abw.txt
        echo "size_thresh=${size_thresh}" >> cfg/config_abw.txt
        echo "distance_thresh=${distance_thresh}" >> cfg/config_abw.txt
        echo "" >> cfg/config_abw.txt
        echo "boundary=${boundary}" >> cfg/config_abw.txt
        echo "" >> cfg/config_abw.txt
        echo "# normal concensus" >> cfg/config_abw.txt
        echo "norm_cont=${norm_cont}" >> cfg/config_abw.txt
        echo "angle_diff=${angle_diff}" >> cfg/config_abw.txt
        echo "" >> cfg/config_abw.txt
        echo "# for merge state" >> cfg/config_abw.txt
        echo "merge=${merge}" >> cfg/config_abw.txt
        echo "plane_distance_thresh=${plane_distance_thresh}" >> cfg/config_abw.txt
        echo "plane_angle_diff=${plane_angle_diff}" >> cfg/config_abw.txt
  
        # segment
        ./bin/train_abw
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
      exper_state=$(echo ${exper_state} | tr -s '[:lower:]' '[:upper:]')
      echo "| ${seeding_method} | ${seeds_count} | ${distance_thresh} | ${size_thresh} | ${boundary} | ${norm_cont} | ${angle_diff} | ${merge} | ${plane_distance_thresh} | ${plane_angle_diff} |">> result/abw_train_results.txt
      python3 ./compare.py ./data/ABW_DATASET/ABW_${exper_state}_GT ${result_dir} --all -t 0.8 -s  >> result/abw_train_results.txt 
      echo " " >> abw_train_results.txt
    done
  done
done
echo " " >> result/abw_train_results.txt