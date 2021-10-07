#!/bin/bash
# remove the txt file if exists to avoid over-appending of data across different simulation runs
rm -f three_link_rand.txt
filename='ia_means_3l.txt'

while read line;
do
  echo "The ia_mean: " $line
  # sim for each rngrun (indep trials)
  for run in $(seq 1 30);
  do
      echo "The run number:" $run
      # run sim
      NS_GLOBAL_VALUE="RngRun=$run""" ./waf --run "scratch/hidden_asymetric -sim_time=50 -ia_mean=$line -outputmode=2" >> three_link_rand.txt
  done
done < $filename
