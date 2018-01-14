# txt2dat.py:  Extracts joint data from the given .txt file to create an 
#              xgraph .dat file.

from __future__ import with_statement
from math       import *
from string     import *
from locale     import format_string

import sys, os

def save_data(outfile,cols,data):
  
  f = open(outfile,"w")
  f.write("TitleText: " + outfile + "\n")
  
  for k in range(len(cols)-1) :
    i = k + 1
    f.write('"' + cols[i] + '"' + "\n")
    for n in range(len(data)) :
      if i < len(data[n][1]) :
        f.write( "%s %s\n" % \
                 (data[n][1][0],data[n][1][i]) )
    f.write("\n")
    
  f.close()
    
  
def load_data(infile,joint,rdamp) :
  
  DPR = 180.0/pi
  
  cols  = ['t']
  data  = []
  time0 = 0.0
  tstep = 0.005
  index = 0
  with open(infile,"r") as f :
    for linen in f :
      (line, eol)  = split(linen, "\n")
      (name, info) = split(line, ' : ')
      if name == joint :
        key_val_pairs = split(info, ', ')
        for key_val in  key_val_pairs :
          (key, val) = split(key_val,'=')
          key   = strip(key)
          val   = strip(val)
          found = False
          ksave = 0
          k     = 0
          while not found and k < len(cols) :
            if cols[k] == key : 
              found = True
              ksave = k
            else : k += 1
          if not found : 
            cols.append(key)
            ksave = k
          if key == "t" : 
            time = val
            if not data : 
              time0 = time 
              data.append( (time,[time]) )
            index = int( (float(time)-float(time0)+ tstep/2.0)/tstep )
            if index > len(data)-1 : 
              for n in range(index-(len(data)-1)) :
                data.append( (time,[time]) )
                time = str(float(time) + tstep)
          else :
            if key == 'p'    : val = format_string("%3d",int(val)*100)
            if key == 'ang1' : val = format_string("%7.2f",float(val)*DPR)
            if key == 'ar1'  : val = format_string("%9.2f",float(val)*DPR)
            if key == 'ar2' :  val = format_string("%9.2f",float(val)*DPR)
            if rdamp :
              if key == 'Td1' : val = format_string("%9.2f",float(val)*DPR)
              if key == 'Td2' : val = format_string("%9.2f",float(val)*DPR)
            if ksave > len(data[index][1])-1 : data[index][1].append(val)
            elif data[index][1][ksave] != val :
              print("error:", time, index, key, ksave, val, data[index][1][ksave])
              print(" line:", linen)

    print("Data columns found:")
    print cols
    
  return cols, data
    
if __name__ == '__main__' :
  
  if len(sys.argv) < 3 :
    print("usage: " + sys.argv[0] + " filename joint [rdamp]")
    sys.exit(0)
  else :
    infile = sys.argv[1]
    joint  = sys.argv[2]
    if len(sys.argv) > 3 : rdamp = True
    else                 : rdamp = False
    
  (name, ext) = split(infile,'.')
  outfile = str.join('.',(name,"dat"))

  (cols, data) = load_data(infile, joint, rdamp)
  
  save_data(outfile,cols,data)
  
