#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f five_link_HN.txt
filename='deltaHN.txt'

while read line;
do
    echo "The delta2 value: " $line
    for run in $(seq 1 100);
    do
        echo "The run number:" $run
        # run sim
        NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/flow_5links -sim_time=100 -ia_mean=0.003 -outputmode=2 -TopologyRun=1" >> five_link_HN.txt
    done
done < $filename
