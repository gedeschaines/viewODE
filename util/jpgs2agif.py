import glob

from PIL import Image
from locale import format_string

import gifmaker

sequence = []
num = -1

for infile in sorted(glob.glob("image_0*.jpg")):
  im = Image.open(infile,"r")
  imc = im.convert('L')
  sequence.append(imc)
  num += 1
  print "Loaded and converted", infile
  
outfile = "images_000_" + format_string("%03d",num) + ".gif"
fp = open(outfile, "wb")
gifmaker.makedelta(fp, sequence)
fp.close