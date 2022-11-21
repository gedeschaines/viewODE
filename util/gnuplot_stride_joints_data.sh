#!/bin/bash

# Execute this script in the ./zout directory containing viewODE stride
# joint debug output file generated by executing viewODE as:
#
#   $ mkdir zout (if it does not already exist)
#   $ python3 viewODE.py 6.0 J:D:Z:0.5:Y:W 1>zout/stride_joints_output.txt
#
# then change to zout subdirectory and invoke this script:
#
#   $ cd zout
#   $ ./gnuplot_stride_joints_data.sh

# Extract joints data from striding joints debug output file.
../util/extract_stride_joints_data.sh

# Invoke gnuplot to render and save plots as JPEG images.
BALL_JOINTS="R_foot-R_ballUniversal L_foot-L_ballUniversal"
ANKLE_JOINTS="R_shin-R_ankleUniversal L_shin-L_ankleUniversal"
KNEE_JOINTS="R_thigh-R_kneeHinge L_thigh-L_kneeHinge"
HIP_JOINTS="pelvis-R_hipUniversal pelvis-L_hipUniversal"
JOINTS_LIST="$BALL_JOINTS $ANKLE_JOINTS $KNEE_JOINTS $HIP_JOINTS"
for J in $JOINTS_LIST
do
  if [ "$J" == "R_thigh-R_kneeHinge" ] || [ "$J" == "L_thigh-L_kneeHinge" ]
  then
    sed -i 's/2:9/7:8/1' "${J}_gnuplot.dat"
  else
    sed -i 's/2:12/7:10/1' "${J}_gnuplot.dat"
  fi
  J_NAME=`echo "$J" | sed s/_//g`
  PLOT_DAT=\'${J}_gnuplot.dat\'
  PLOT_IMG=\'${J}.jpg\'
  SET_TITLE="set title '$J_NAME Joint Rotation Angle (ang) and Rate (ar)'"
  SET_YLABEL="set ylabel 'Rotation Angle (deg), Rate (deg/sec)'"
  PLOT_DAT_CMDS="load $PLOT_DAT; set grid; $SET_TITLE; $SET_YLABEL; replot"
  PLOT_IMG_CMDS="set term push; set term jpeg; set output $PLOT_IMG; replot"
  taskset -c 2 gnuplot -e "$PLOT_DAT_CMDS; $PLOT_IMG_CMDS; set term pop; quit"
done
