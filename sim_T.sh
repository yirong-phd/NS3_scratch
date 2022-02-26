#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f five_link_T.txt
filename='time.txt'


while read line;
do
  echo "The sim_time: " $line
  # sim for each rngrun (indep trials)
  for run in $(seq 1 100);
  do
      echo "The run number:" $run
      # run sim
      NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/flow_5links -sim_time=$line -ia_mean=0.003 -outputmode=2" >> five_link_T.txt
  done
done < $filename
