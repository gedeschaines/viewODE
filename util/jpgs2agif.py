import glob

from PIL import Image
from locale import format_string

import gifmaker

sequence = []
num = -1

for infile in sorted(glob.glob("img_0*.jpg")):
  im = Image.open(infile,"r")
  imc = im.convert('L')
  sequence.append(imc)
  num += 1
  print "Loaded and converted", infile

if num < 0:
  print("No JPEG image files loaded.")
else:
  outfile = "anim_0000_" + format_string("%04d",num) + ".gif"
  fp = open(outfile, "wb")
  if fp:
    print("Writing animated GIF file %s" % outfile)
    gifmaker.makedelta(fp, sequence)
    fp.close
    print("Done.")