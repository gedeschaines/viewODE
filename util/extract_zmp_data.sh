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
#
# and invoke gnuplot to display cm, cop and zmp data:
#
#   $ gnuplot -p cmpos_gnuplot.dat
#   $ gnuplot -p cmvel_gnuplot.dat
#   $ gnuplot -p cmacc_gnuplot.dat
#   $ gnuplot -p copR_gnuplot.dat
#   $ gnuplot -p copL_gnuplot.dat
#   $ gnuplot -p zmp_gnuplot.dat
#
# NOTE: The previous two steps are performed by executing
# the ./util/gnuplot_zmp_data.sh script from within the
# ./zout subdirectory.

ZMP_DATA_TXT_FILE=zmp_output.txt

ZMP_LIST="cmpos cmvel cmacc copR copL zmp"

for P in $ZMP_LIST
do
  grep "$P" "$ZMP_DATA_TXT_FILE" > "${P}.txt"
done

for P in $ZMP_LIST
do
  python2 ../util/txt2dat.py "${P}.txt" "$P" "0"
done
