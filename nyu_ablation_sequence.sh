#!/bin/bash
# Program:
#     This program is to extract planes from a sequence and compared 
#
# History:
# 20200911	BIgRunner	V0.0.1
read -p "Please put seeds in each cell: " seeds_per_cell
read -p "Please set size of each cell: " cell_size
read -p "Please input distance thresh in growing: " distance_thresh
read -p "Please input size thresh in growing: " size_thresh
read -p "With boundary? (1 for yes, 0 for no): " boundary
read -p "With normal contency check? (1 for yes, 0 for no): " norm_cont
read -p "Please input normal difference in growing: " angle_diff
read -p "With merge? (1 for yes, 0 for no): " merge
read -p "Please input distance thresh in merging: " plane_distance_thresh
read -p "Please input normal difference in merging: " plane_angle_diff

result_dir=./result/nyu_${seeds_per_cell}_${cell_size}_${distance_thresh}_${size_thresh}_${boundary}_${norm_cont}_${angle_diff}_${merge}_${plane_distance_thresh}_${plane_angle_diff}

if [ ! -d result_dir ]; then
  mkdir -p ${result_dir}
fi

for file in ./data/NYU_DEPTH/*
do
  num=${file##*_}
  num=${num%.png}
  depth_file=./data/NYU_DEPTH/nyu_${num}.png
  color_file=./data/NYU_COLOR/nyu_${num}.png
  echo "# Depth file" > config_nyu.txt
  echo "depth=${depth_file}" >> config_nyu.txt
  echo "color=${color_file}" >> config_nyu.txt
  echo "save_path=${result_dir}/nyu_${num}_MS.png" >> config_nyu.txt
  echo "" >> config_nyu.txt
  echo "# camera" >> config_nyu.txt
  echo "camera.r0=196.444" >> config_nyu.txt
  echo "camera.c0=275.045" >> config_nyu.txt
  echo "camera.fr=582.691" >> config_nyu.txt
  echo "camera.fc=582.624" >> config_nyu.txt
  echo "camera.scale=1000.0" >> config_nyu.txt
  echo "" >> config_nyu.txt
  echo "# parameters for sementation" >> config_nyu.txt
  echo "seeds_per_cell=${seeds_per_cell}" >> config_nyu.txt
  echo "cell_size=${cell_size}" >> config_nyu.txt
  echo "size_thresh=${size_thresh}" >> config_nyu.txt
  echo "distance_thresh=${distance_thresh}" >> config_nyu.txt
  echo "" >> config_nyu.txt
  echo "# boundary" >> config_nyu.txt
  echo "boundary=${boundary}" >> config_nyu.txt
  echo "" >> config_nyu.txt
  echo "# normal concensus" >> config_nyu.txt
  echo "norm_cont=${norm_cont}" >> config_nyu.txt
  echo "angle_diff=${angle_diff}" >> config_nyu.txt
  echo "" >> config_nyu.txt
  echo "# for merge state" >> config_nyu.txt
  echo "merge=${merge}$" >> config_nyu.txt
  echo "plane_distance_thresh=${plane_distance_thresh}" >> config_nyu.txt
  echo "plane_angle_diff=${plane_angle_diff}" >> config_nyu.txt
	
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
python3 ./compare.py ./data/NYU_GROUND_TRUTH ${result_dir} --all -t 0.5 -s 
