#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f five_link.txt
filename='ia_means_3l.txt' #use the same ia_means with 3-link test case

while read line;
do
  echo "The ia_mean: " $line
  # sim for each rngrun (indep trials)
  for run in $(seq 1 30);
  do
      echo "The run number:" $run
      # run sim
      NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/flow_5links -sim_time=50 -ia_mean=$line -prop_loss=2 -outputmode=2" >> five_link.txt
  done
done < $filename
