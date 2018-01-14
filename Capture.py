# PyODE Capture.py:  captures framebuffer images.

# Originally by Gary Deschaines

import sys

from locale import format_string

try:
  import OpenGL
  OpenGL.USE_FREEGLUT = False
  from OpenGL.GL import *
  from OpenGL.GLU import *
  from OpenGL.GLUT import *
except:
  print("Error: This module requires PyOpenGL !!")
  sys.exit()
  
try:
  from PIL import Image
except:
  print("Error: This module requires PIL !!")
  sys.exit()
  
class Capture:
    
  def __init__(self, renderer, frame_time, filename):
    """ Constructor.
        Capture -- captures images from the Frame buffer
                    
        Initialize the ODE figure rendered solid selector
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
      scount = format_string("%04d",self.count)
      file   = "./zimages/" + self.filename + "_" + scount + ".jpg"
      image.save(file,"JPEG")
    
      # Increment image counter
      self.count += 1
