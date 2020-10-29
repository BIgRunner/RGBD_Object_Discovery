#!/bin/bash
# Program:
#     This program is to extract planes from a sequence and compared 
#
# History:
# 20200911	BIgRunner	V0.0.1
read -p "Please input experiment state: " exper_state
read -p "Sequence start index: " start
read -p "Sequence end index: " end
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

result_dir=./result/abw_${seeds_per_cell}_${cell_size}_${distance_thresh}_${size_thresh}_${boundary}_${norm_cont}_${angle_diff}_${merge}_${plane_distance_thresh}_${plane_angle_diff}

if [ ! -d result_dir ]; then
  mkdir -p ${result_dir}
fi

for num in $(seq ${start} $((${end}-1)))
do
  depth_file=./data/abw_depth/abw_${exper_state}_${num}.png
  color_file=./data/abw_color/abw_${exper_state}_${num}.png
  echo "# Depth file" > config_abw.txt
  echo "depth=${depth_file}" >> config_abw.txt
  echo "color=${color_file}" >> config_abw.txt
  echo "save_path=${result_dir}/abw_${exper_state}_${num}_ms-seg.png" >> config_abw.txt
  echo "" >> config_abw.txt
  echo "# camera" >> config_abw.txt
  echo "camera.r0=255.0" >> config_abw.txt
  echo "camera.c0=255.0" >> config_abw.txt
  echo "camera.fr=2337.212" >> config_abw.txt
  echo "camera.fc=1610.982" >> config_abw.txt
  echo "camera.scale=773.545" >> config_abw.txt
  echo "" >> config_abw.txt
  echo "# parameters for sementation" >> config_abw.txt
  echo "seeds_per_cell=${seeds_per_cell}" >> config_abw.txt
  echo "cell_size=${cell_size}" >> config_abw.txt
  echo "distance_thresh=${distance_thresh}" >> config_abw.txt
  echo "size_thresh=${size_thresh}" >> config_abw.txt
  echo "" >> config_abw.txt
  echo "boundary=${boundary}" >> config_abw.txt
  echo "" >> config_abw.txt
  echo "# normal concensus" >> config_abw.txt
  echo "norm_cont=${norm_cont}" >> config_abw.txt
  echo "angle_diff=${angle_diff}" >> config_abw.txt
  echo "" >> config_abw.txt
  echo "# for merge state" >> config_abw.txt
  echo "merge=${merge}" >> config_abw.txt
  echo "plane_distance_thresh=${plane_distance_thresh}" >> config_abw.txt
  echo "plane_angle_diff=${plane_angle_diff}" >> config_abw.txt
	
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
python3 ./compare.py ./data/ABW_${exper_state}_GT ${result_dir} --all -t 0.8 -s 
