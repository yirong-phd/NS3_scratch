#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f ten_link_rand.txt
rm -f topology_cg.txt
filename='ia_means.txt'

for T_run in $(seq 1 2);
do
  echo "The Topology: " $T_run
  while read line;
  do
    echo "The ia_mean: " $line
    # sim for each rngrun (indep trials)
    for run in $(seq 1 30);
    do
        echo "The run number:" $run
        # run sim
        NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/RandGraph -sim_time=50 -ia_mean=$line -outputmode=2 -TopologyRun=$T_run" >> ten_link_rand.txt
        if (($run = 1)) then;
          echo "Compute CG:"
          NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/RandGraph -sim_time=50 -ia_mean=$line -outputmode=0 -TopologyRun=$T_run" >> topology_cg.txt
        fi
    done
  done < $filename
done
