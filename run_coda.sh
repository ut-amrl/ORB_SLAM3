#!/bin/bash

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <mode> <seq> <num_trial>"
  echo "<mode> should be 'stereo' or 'mono'"
  exit 1
fi

mode=$1 # stereo or mono
seq=$2  # sequence number
num_trial=$3 # number of trials

project_dir=/home/dongmyeong/Projects/AMRL/vio-evaluation
dataset_dir=/home/dongmyeong/Projects/AMRL/CODa

voc_file=$project_dir/ORB_SLAM3/Vocabulary/ORBvoc.txt
img_dir=$dataset_dir/2d_raw
time_dir=$dataset_dir/timestamps

for (( trial=1; trial<=$num_trial; trial++ ))
do
  # Configuration and output file setup based on mode
  if [ "$mode" == "stereo" ]; then
    config_file=$project_dir/ORB_SLAM3/Examples/Stereo/configs/CODa-01.yaml
    exec_file=$project_dir/ORB_SLAM3/Examples/Stereo/stereo_coda
    out_file=dataset-CODa$(printf "%02d" $seq)-stereo_$trial
  elif [ "$mode" == "mono" ]; then
    config_file=$project_dir/ORB_SLAM3/Examples/Monocular/configs/CODa-01.yaml
    exec_file=$project_dir/ORB_SLAM3/Examples/Monocular/mono_coda
    out_file=dataset-CODa$(printf "%02d" $seq)-mono_$trial
  else
    echo "Invalid mode: $mode"
    echo "Mode should be either 'stereo' or 'mono'"
    exit 1
  fi

  # Execute the command with the provided sequence number
  echo "Executing trial $trial..."
  $exec_file $voc_file $config_file $img_dir $time_dir $seq $out_file

  mv f_$out_file.txt $project_dir/benchmark/trajectories/CODa/ORB_SLAM3/
  mv lost_$out_file.txt $project_dir/benchmark/trajectories/CODa/ORB_SLAM3/
  echo "Output for trial $trial moved to: $project_dir/benchmark/trajectories/CODa/ORB_SLAM3"
done

echo "Completed $num_trial trials."

