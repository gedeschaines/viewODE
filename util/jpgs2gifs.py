      
import glob
from os import path
from PIL import Image

for infile in sorted(glob.glob("*.jpg")):
  file, ext = path.splitext(infile)
  im_jpg = Image.open(infile)
  im_jpg.save(file + ".gif")
