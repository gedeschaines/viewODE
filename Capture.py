# Capture.py:
#
# Captures framebuffer images.
#
# Originally by Gary Deschaines, 2009.

from sys import exit
from locale import format_string

#
# Import OpenGL and PIL modules for capturing images from back framebuffer.

try:
  import OpenGL
  OpenGL.USE_FREEGLUT = False
  from OpenGL.GL import *
  from OpenGL.GLU import *
  from OpenGL.GLUT import *
except:
  print("Error: This module requires PyOpenGL !!")
  exit()
  
try:
  from PIL import Image
except:
  print("Error: This module requires PIL !!")
  exit()
  
class Capture:
    
  def __init__(self, renderer, frame_time, filename):
    """
    Constructor.
    Capture -- captures images from the framebuffer.
                    
    Instantiate a viewODE rendered image capture object.
    """
    
    self.renderer     = renderer
    self.frame_time   = frame_time 
    self.filename     = filename
    self.msec_counter = frame_time
    self.count        = 0
    
  def zeroMsecCounter(self):
    
    self.msec_counter = 0
    
  def resetMsecCounter(self):
    
    self.msec_counter = self.frame_time
    
  def updateMsecCounter(self, dt_msec):
    
    self.msec_counter += dt_msec
    
  def saveImage(self):
    
    if  self.msec_counter == self.frame_time :
      
      # Zero millisecond counter
      self.msec_counter = 0 
      
      # Get image size
      imgx = 0
      imgy = 0
      imgw = self.renderer.width
      imgh = self.renderer.height
      size = (imgw,imgh)
    
      # Read color pixels from back framebuffer into data array
      glReadBuffer(GL_BACK)
      glPixelStorei(GL_PACK_ALIGNMENT, 1)
      data = glReadPixelsub(imgx, imgy, imgw, imgh, GL_RGBA)
      if hasattr(data, 'shape') :
        assert data.shape == (imgw,imgh,4), \
          """Got back array of shape %r, expected %r""" % \
             (data.shape, (size,4))
     
      # Get decoded image from data array
      image = Image.frombuffer( "RGB", size, data,   \
                                "raw", "RGBA", 0, -1 ) # Decoder
                                
      # Save image as a JPEG file
      if image.mode in ("RGBA"): image = image.convert("RGB")
      scount = format_string("%04d",self.count)
      file   = "./zimg/" + self.filename + "_" + scount + ".jpg"
      image.save(file)
    
      # Increment image counter
      self.count += 1
