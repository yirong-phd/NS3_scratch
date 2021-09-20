#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f ten_link_T.txt
filename='time.txt'

for T_run in $(seq 1 1);
do
  echo "The Topology: " $T_run
  while read line;
  do
    echo "The sim_time: " $line
    # sim for each rngrun (indep trials)
    for run in $(seq 1 30);
    do
        echo "The run number:" $run
        # run sim
        NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/RandGraph -sim_time=$line -ia_mean=0.01 -outputmode=2 -TopologyRun=$T_run" >> ten_link_T.txt
    done
  done < $filename
done
