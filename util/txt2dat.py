#!/usr/bin/env python2
# txt2dat.py:  Extracts joint data from the given .txt file to create
#              xgraph or gnuplot .dat file.

from __future__ import with_statement
from math import *
from string import *
from locale import format_string

import sys


def save_data_xgraph(outfile, joint, stride, cols, data):
    colors = {0: 'black', 1: 'white', 2: 'red', 3: 'blue', 4: 'green', 5: 'violet', 6: 'orange',
              7: 'yellow', 8: 'pink', 9: 'cyan', 10: 'light-gray', 11: 'dark-grey', 12: 'fuchia',
              13: 'aqua', 14: 'navy', 15: 'gold'}

    f = open(outfile, "w")
    if stride:
        f.write("Title = " + joint + ": Hip, Knee and Ankle Rotation (ang), Hip Rotation Rate (angv)\n")
    else:
        f.write("Title = " + outfile + "\n")
    f.write("title_x = Time (sec)\n")
    if stride:
        f.write("title_y = Ang (deg), AngV (deg/sec)\n")

    # Data
    for k in range(len(cols) - 1):
        i = k + 1
        if i < len(data[0][1]):
            f.write("color = " + colors[i + 1] + "\n")
        for n in range(len(data)):
            if i < len(data[n][1]):
                f.write("%s %s\n" %
                        (data[n][1][0], data[n][1][i]))
        if i < len(cols) - 1:
            f.write("next\n")

    # Legend

    x1 = 0.1
    x2 = 0.3
    y = 1.0

    f.write("Text " + " ".join([str(x1), str(y - 0.2)]) + "\n")
    f.write("LEGEND\n")
    f.write("End_Text\n")

    for k in range(len(cols) - 1):
        i = k + 1
        line = " ".join([str(x1), str(y), str(x2), str(y)])
        f.write("color = " + colors[i + 1] + "\n")
        f.write("TITlE_LEGEND_LINE " + line + "\n")
        f.write("Text " + " ".join([str(x2 + 0.1), str(y)]) + "\n")
        if stride and cols[i].find('_') > 0:
            # use '_' to indicate start of subscript text
            (str1, str2) = str.split(cols[i], '_')
            if str1 == 'h':
                str1 = "Hip"
            elif str1 == 'k':
                str1 = "Knee"
            elif str1 == 'a':
                str1 = "Ankle"
            cols[i] = str1 + "_" + str2
        f.write(" " + cols[i] + "\n")
        f.write("End_Text\n")
        y += 0.2

    f.close()


def save_data_gnuplot(outfile, joint, stride, cols, data):
    f = open(outfile, "w")
    f.write('# File: "' + outfile + '"\n')
    if stride:
        f.write('set title "' + joint + ': Hip, Knee and Ankle Rotation (ang); Hip Rotation Rate (angv)"\n')
    f.write('set xlabel "Time (sec)"\n')
    if stride:
        f.write('set ylabel "Ang (deg), AngV (deg/sec)"\n')
    f.write('set key left top Left\n')
    f.write("$JointData << EOD\n")
    for i in range(len(cols)):
        if cols[i].find('_') < 1:
            continue
        # use '_' to indicate start of subscript text
        (str1, str2) = str.split(cols[i], '_')
        if stride:
            if str1 == 'h':
                str1 = "Hip"
            elif str1 == 'k':
                str1 = "Knee"
            elif str1 == 'a':
                str1 = "Ankle"
        cols[i] = str1 + "_{" + str2 + "}"
    f.write(' '.join(cols) + "\n")
    for n in range(len(data)):
        f.write("%s %s\n" % (data[n][1][0], ' '.join(data[n][1][1:])))
    f.write("EOD\n")
    f.write("set termopt enhanced\n")
    f.write("plot for [k=2:%s] $JointData using 1:k with lines title columnhead(k)\n" % (str(len(cols))))
    f.close()


def load_data(infile, joint, rdamp):
    DPR = 180.0 / pi

    cols = ['t']
    data = []
    time0 = 0.0
    tstep = 0.0025  # NOTE: This must match tstep in viewODE.py
    index = 0
    with open(infile, "r") as f:
        for linen in f:
            (line, eol) = str.split(linen, "\n")
            if line.find(' : ') < 1:
                continue
            (name, info) = str.split(line, ' : ')
            if name == joint:
                key_val_pairs = str.split(info, ', ')
                for key_val in key_val_pairs:
                    (key, val) = str.split(key_val, '=')
                    key = str.strip(key)
                    val = str.strip(val)
                    found = False
                    ksave = 0
                    k = 0
                    while not found and k < len(cols):
                        if cols[k] == key:
                            found = True
                            ksave = k
                        else:
                            k += 1
                    if not found:
                        cols.append(key)
                        ksave = k
                    if key == "t":
                        time = val
                        if not data:
                            time0 = time
                            data.append((time, [time]))
                        index = int((float(time) - float(time0) + tstep / 2.0) / tstep)
                        if index > len(data) - 1:
                            for n in range(index - (len(data) - 1)):
                                data.append((time, [time]))
                                time = str(float(time) + tstep)
                    else:
                        if key == 'p':
                            val = format_string("%3d", int(val) * 100)
                        if key == 'ang':
                            val = format_string("%7.2f", float(val) * DPR)
                        if key == 'ang1':
                            val = format_string("%7.2f", float(val) * DPR)
                        if key == 'ang2':
                            val = format_string("%7.2f", float(val) * DPR)
                        if key == 'ar':
                            val = format_string("%9.2f", float(val) * DPR)
                        if key == 'ar1':
                            val = format_string("%9.2f", float(val) * DPR)
                        if key == 'ar2':
                            val = format_string("%9.2f", float(val) * DPR)
                        if rdamp:
                            if key == 'Td1':
                                val = format_string("%9.2f", float(val) * DPR)
                            if key == 'Td2':
                                val = format_string("%9.2f", float(val) * DPR)
                        if ksave > len(data[index][1]) - 1:
                            data[index][1].append(val)
                        elif data[index][1][ksave] != val:
                            print("error:", time, index, key, ksave, val, data[index][1][ksave])
                            print(" line:", linen)

        print("Data columns found:")
        print(cols)

    return cols, data


if __name__ == '__main__':

    if len(sys.argv) < 4:
        print("usage: " + sys.argv[0] + " filename joint 0|1 [rdamp]")
        sys.exit(0)
    else:
        infile = sys.argv[1]
        joint = sys.argv[2]
        xgraph = sys.argv[3]
        if len(sys.argv) > 4:
            rdamp = True
        else:
            rdamp = False

    (name, ext) = str.split(infile, '.')

    (cols, data) = load_data(infile, joint, rdamp)

    if joint == "right_leg" or joint == "left_leg":
        stride = True
        (side, _) = str.split(joint, '_')
        joint = side[0].upper() + side[1:] + " Leg"
    else:
        stride = False

    if int(xgraph) == 1:
        outfile = str.join('.', (name + '_xgraph', 'dat'))
        save_data_xgraph(outfile, joint, stride, cols, data)
    else:
        outfile = str.join('.', (name + '_gnuplot', 'dat'))
        save_data_gnuplot(outfile, joint, stride, cols, data)
