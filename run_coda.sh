#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <mode> <seq>"
  echo "<mode> should be 'stereo' or 'mono'"
  exit 1
fi

mode=$1 # stereo or mono
seq=$2  # sequence number

project_dir=/home/dongmyeong/Projects/AMRL/vio-evaluation
dataset_dir=/home/dongmyeong/Projects/AMRL/CODa

voc_file=$project_dir/ORB_SLAM3/Vocabulary/ORBvoc.txt
img_dir=$dataset_dir/2d_raw
time_dir=$dataset_dir/timestamps

# Configuration and output file setup based on mode
if [ "$mode" == "stereo" ]; then
  config_file=$project_dir/ORB_SLAM3/Examples/Stereo/configs/CODa-01.yaml
  exec_file=$project_dir/ORB_SLAM3/Examples/Stereo/stereo_coda
  out_file=dataset-CODa$(printf "%02d" $seq)-stereo
elif [ "$mode" == "mono" ]; then
  config_file=$project_dir/ORB_SLAM3/Examples/Monocular/configs/CODa-01.yaml
  exec_file=$project_dir/ORB_SLAM3/Examples/Monocular/mono_coda
  out_file=dataset-CODa$(printf "%02d" $seq)-mono
else
  echo "Invalid mode: $mode"
  echo "Mode should be either 'stereo' or 'mono'"
  exit 1
fi

# Execute the command with the provided sequence number
$exec_file $voc_file $config_file $img_dir $time_dir $seq $out_file

# move the output file to the trajectory folder
echo "$project_dir/benchmark/trajectories/CODa/ORB_SLAM3/f_$out_file.txt"
mv f_$out_file.txt $project_dir/benchmark/trajectories/CODa/ORB_SLAM3/

