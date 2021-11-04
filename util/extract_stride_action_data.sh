#!/bin/bash

# Execute this script in ./zout subdirectory containing viewODE stride
# action debug output file generated by executing viewODE as shown below
# starting in directory containing the viewODE.py file:
#
#   $ mkdir zout
#   $ python viewODE.py 1>zout/stride_action_output.txt
#   $ Z
#   $ H
#   $ Y
#   $ W
#   $ Esc
#   $ cd zout
#   $ ../util/extract_stride_action_data.sh
#   $ gnuplot -p right_leg_gnuplot.dat
#   $ gnuplot -p left_leg_gnuplot.dat

ACTION_DATA_TXT_FILE=stride_action_output.txt

LEG_LIST="right_leg left_leg"

for LEG in $LEG_LIST
do
  grep $LEG $ACTION_DATA_TXT_FILE > ${LEG}.txt
done

for LEG in $LEG_LIST
do
  python2 ../util/txt2dat.py ${LEG}.txt $LEG "0"
done
