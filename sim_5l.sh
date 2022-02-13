#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f five_link_HN.txt
filename='deltaHN.txt'

while read line1;
do
  while read line2;
  do
    echo "The delta2 value: " $line1
    echo "The delta4 value: " $line2
    for run in $(seq 1 30);
    do
        echo "The run number:" $run
        # run sim
        NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/flow_5links -sim_time=50 -ia_mean=0.01 -prop_loss=2 -delta2=$line1 -delta4=$line2 -outputmode=2 -TopologyRun=1" >> five_link_HN.txt
    done
  done < $filename
done < $filename
