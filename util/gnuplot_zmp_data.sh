#!/bin/bash

# Execute this script in ./zout subdirectory containing viewODE
# walking action debug ZMP output file generated by executing
# viewODE as shown below starting in directory containing the
# viewODE.py file:
#
#   $ mkdir zout (if it does not already exist)
#   $ python3 viewODE.py 6.0 H:D:Z:0.5:W 1>zout/zmp_output.txt
#
# then change to zout subdirectory and invoke this script:
#
#   $ cd zout
#   $ ../util/gnuplot_zmp_data.sh

# Extract data from walking action debug ZMP output file.
../util/extract_zmp_data.sh

# Invoke gnuplot to render and save plots as JPEG images.
PLOT_LIST="cmpos cmvel cmacc copR copL zmp"
for P in $PLOT_LIST
do
  if [ "$P" == "copL" ]
  then
    sed -i 's/ top / bottom /1' "${P}_gnuplot.dat"
  fi
  PLOT_DAT=\'${P}_gnuplot.dat\'
  PLOT_IMG=\'${P}.jpg\'
  PLOT_DAT_CMDS="load $PLOT_DAT; set grid; replot"
  PLOT_IMG_CMDS="set term push; set term jpeg; set output $PLOT_IMG; replot"
  taskset -c 2 gnuplot -e "$PLOT_DAT_CMDS; $PLOT_IMG_CMDS; set term pop; quit"
done
