#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f 5l_HN_top7.txt
filename='ia_means_top7.txt'

while read line;
do
    echo "The ia means for top7: " $line
    for run in $(seq 1 100);
    do
        echo "The run number:" $run
        # run sim
        NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/flow_5links -sim_time=50 -prop_loss=1 -ia_mean=$line -outputmode=2 -top=7" >> 5l_HN_top7.txt
    done
done < $filename
