# Select.py
#
# viewODE figure rendered solid selection definition module.
#
# Originally by Gary Deschaines, 2009.

import sys
import os

from math import *

#
# Import OpenGL modules for rendering and selection.

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
  from vecMath import *
except:
  print("Error: vecMath not installed properly !!")

class Select:
    
  def __init__(self, renderer, renderPick):
    """ 
    Select Constructor -- ODE figure rendered solids selection.
                    
    Initialize the ODE figure rendered solid selector.
    """
    
    self.renderer         = renderer
    self.renderPick       = renderPick
    self.debug            = False
    self.is_selecting     = False
    self.last_picked      = None
    self.picked_body      = None
    self.picked_body_info = []
    self.prev_n_vec       = []
    self.prev_f_vec       = []
    self.n_vec            = []
    self.f_vec            = []
    self.c_vec            = []
    self.c_uvec           = []
    self.c_vmag           = 0.0
    self.SELECTVXYZ       = []
    self.SELECTBODIES     = []
    
  def debugOn(self)     : self.debug = True
  def debugOff(self)    : self.debug = False
  def toggleDebug(self) : self.debug = not self.debug
    
  def isSelecting(self):
    
    return self.is_selecting
    
  def numSelectedBodies(self):
    
    return len(self.SELECTBODIES)
    
  def getPickedBody(self):
    
    return self.picked_body
    
  def getPickedBodyInfo(self):
        
    return self.picked_body_info
    
  def pickClosestSelectBody(self):
    """
    Pick the closest body among those in the selected body list.
    """

    if self.SELECTBODIES :
      
      # Pick the selected body which is closest to 
      # the near clipping plane
    
      closest_near = self.SELECTBODIES[0][2]
      for (b, near, far) in self.SELECTBODIES:
        if near < closest_near :
          closest_body = b
          closest_near = near
          closest_far  = far
      
      if self.debug :
        print("closest solid=%s, near=%f" % \
              (closest_body.solid.label,closest_near) )
              
      closest_body_info = ( closest_body, \
                            closest_near, \
                            closest_far   )

      closest_body.solid.wire = 1
      
      self.SELECTBODIES = []                         
      self.SELECTBODIES.append( (closest_body_info) )
      self.last_picked      = self.picked_body
      self.picked_body      = closest_body      
      self.picked_body_info = closest_body_info
      
  def getPrevNvecFvec(self):

      return ( (self.prev_n_vec, self.prev_f_vec) )
      
  def getSelectNvecFvec(self):

      return ( (self.n_vec, self.f_vec) )

  def getSelectCvmagCuvec(self):

      return ( (self.c_vmag, self.c_uvec) )
      
  def calcSelectNvecFvecCvec(self):
        
    if len(self.SELECTVXYZ) == 8:
      
      # Determine center-line vector of selection volume; 
      # from near clipping plane to far clipping plane
    
      n_vec = [0.0, 0.0, 0.0]
      f_vec = [0.0, 0.0, 0.0]
      for i in range(4):
        j = i + 4
        n_vec[0] = n_vec[0] + self.SELECTVXYZ[i][0]
        n_vec[1] = n_vec[1] + self.SELECTVXYZ[i][1]
        n_vec[2] = n_vec[2] + self.SELECTVXYZ[i][2]
        f_vec[0] = f_vec[0] + self.SELECTVXYZ[j][0]
        f_vec[1] = f_vec[1] + self.SELECTVXYZ[j][1]
        f_vec[2] = f_vec[2] + self.SELECTVXYZ[j][2]
      n_vec = vecMulS(n_vec, 1.0/4.0)
      f_vec = vecMulS(f_vec, 1.0/4.0)
    
      if self.debug :
        print("n_vec(x,y,z} = (%f | %f | %f)" % \
              (n_vec[0], n_vec[1], n_vec[2]) )
        print("f_vec(x,y,z} = (%f | %f | %f)" % \
              (f_vec[0], f_vec[1], f_vec[2]) )
              
      self.prev_n_vec = self.n_vec
      self.prev_f_vec = self.f_vec
      
      self.n_vec = n_vec
      self.f_vec = f_vec
    
      # Compute the direction vector and magnitude of 
      # the center-line vector
    
      c_vec = vecSub(f_vec, n_vec)
                
      if self.debug :
        print("c_vec(x,y,z} = (%f | %f | %f)" % \
              (c_vec[0], c_vec[1], c_vec[2]) )
              
      self.c_vec = c_vec
      
      c_mag = vecMag(c_vec)
      c_vec = vecMulS(c_vec, 1.0/c_mag)
    
      if self.debug :
        print("c_mag, c_vec(x,y,z} = %f, (%f | %f | %f)" % \
              (c_mag, c_vec[0], c_vec[1], c_vec[2]) )
              
      self.c_uvec = c_vec
      self.c_vmag = c_mag
     
  def glSelectWithCallback(self, x, y, callback, \
                              xsize = 5, ysize = 5, \
                              buffer_size = 512):
    """ 
    Replacement for glSelectWithCallback function which is no longer included 
    in the PyOpenGL package.

    @param x: Window x coordinate for center of the pick box.
    @type  x: int
    @param y: Window y coordinate for center of the pick box.
    @type  y: int
    @param callback: Render callback, taking zero arguments, which performs
                     pick-mode rendering.
    @type  callback: callable Python object
    @param xsize: The x dimension of the pick box (default 5).
    @type  xsize: int
    @param ysize: The y dimension of the pick box (default 5).
    @type  ysize: int
    @param buffer_size: Number of bytes allocated for pick results buffer
                        (default 512).
    @type  buffer_size: int

    @return: A tuple (possibly empty) of the form:
    
             (minimumzdepth, maximumzdepth, (name, name, name,...),...)
             
             where:
          
             minimumzdepth, maximumzdepth -- 0.0 to 1.0 corresponding to
             depth from near to far planes of viewing volume frustum.

             If you want physical depth, multiply that by frustum depth
             and add your near clipping plane.  This is valid only for
             orthographic projections since depth values are non-linear
             for perspective projectons.
             
             name -- name (integer) used in calls to glPushName(int).
    @rtype: tuple
    """
    viewport = glGetIntegerv(GL_VIEWPORT)
    previousprojmatrix = glGetDoublev(GL_PROJECTION_MATRIX)
    glSelectBuffer(buffer_size)
    glRenderMode(GL_SELECT)
    glInitNames()
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPickMatrix( float(x), float(viewport[3] - y), \
                   xsize, ysize, viewport            )
    glMultMatrixd(previousprojmatrix)
    glMatrixMode(GL_MODELVIEW)
    callback()
    glFlush()
    glMatrixMode(GL_PROJECTION)
    glLoadMatrixd(previousprojmatrix)
    glMatrixMode(GL_MODELVIEW)
    
    return glRenderMode(GL_RENDER)

  def drawSelectVolume(self):
    """
    Draw a selection rectangle or volume box.
    """
    if len(self.SELECTVXYZ) == 4 or \
       len(self.SELECTVXYZ) == 8 :
      glPushMatrix()
      glRotatef(self.renderer.rotationY, 0.0, 1.0, 0.0)
      glRotatef(self.renderer.rotationX, 1.0, 0.0, 0.0) 
      glColor3f(1.0, 1.0, 1.0)
      glBegin(GL_LINE_LOOP)
      for i in [0,1,2,3] :
        glVertex3f( self.SELECTVXYZ[i][0], \
                    self.SELECTVXYZ[i][1], \
                    self.SELECTVXYZ[i][2]  )
      glEnd()
      if len(self.SELECTVXYZ) == 8 :
        glBegin(GL_LINE_LOOP)
        for j in [4,5,6,7] :
          glVertex3f( self.SELECTVXYZ[j][0], \
                      self.SELECTVXYZ[j][1], \
                      self.SELECTVXYZ[j][2]  )
        glEnd()
        glBegin(GL_LINES)
        for i in [0,1,2,3] :
          j = i + 4
          glVertex3f( self.SELECTVXYZ[i][0], \
                      self.SELECTVXYZ[i][1], \
                      self.SELECTVXYZ[i][2]  )
          glVertex3f( self.SELECTVXYZ[j][0], \
                      self.SELECTVXYZ[j][1], \
                      self.SELECTVXYZ[j][2]  )
        glEnd()
      glPopMatrix()
  
  def clearSelectVolume(self):
    
    self.is_selecting = False
    self.prev_n_vec   = []
    self.prev_f_vec   = []
    self.n_vec        = []
    self.f_vec        = []
    self.c_vec        = []
    self.c_uvec       = []
    self.c_vmag       = 0.0
    self.SELECTVXYZ   = []
    
    self.clearSelectBodies()
    
  def clearSelectBodies(self):
    
    if self.picked_body :
      self.picked_body.solid.wire = 0
      self.picked_body            = None
      self.picked_body_info       = []
    
    if self.SELECTBODIES : 
      self.SELECTBODIES = []
    
  def getSelectVolume(self,n,mx,my):
    """ 
    Get selection rectangle or volume box from mouse windows coordinates (mx,my).
    """
   
    # Using default x and y dimensions (5x5) of the pick box
    # for the x and y size of the select volume  
    dx = [-2.5,  2.5, 2.5, -2.5, -2.5,  2.5, 2.5, -2.5]
    dy = [-2.5, -2.5, 2.5,  2.5, -2.5, -2.5, 2.5,  2.5]
    z  = [ 0.0,  0.0, 0.0,  0.0,  1.0,  1.0, 1.0,  1.0]
    # Get viewport and the projection and modelview matrices 
    # for UnProject
    viewport = glGetIntegerv(GL_VIEWPORT) 
    projmatrix = glGetDoublev(GL_PROJECTION_MATRIX)
    glPushMatrix()
    glRotatef(self.renderer.rotationY, 0.0, 1.0, 0.0)
    glRotatef(self.renderer.rotationX, 1.0, 0.0, 0.0)
    mvmatrix = glGetDoublev(GL_MODELVIEW_MATRIX)
    glPopMatrix()
    # Convert mouse (x,y) to viewport (vx,vy)
    if self.debug :
      print("Coordinates at mouse cursor (%4d | %4d)" % (mx,my) )
    vx = float(mx)
    vy = float(viewport[3] - my)
    # Convert select view volume coordinates (x,y,z) to world 
    # coordinates (wx,wy,wz)  
    selectvol = []
    if self.debug :
      if ( n == 4 ) :
        print("Select rectangle vertices in world coordinates:")
      elif ( n == 8 ) :
        print("Select volume vertices in world coordinates:")
    for k in range(n):
      x = vx + dx[k]
      y = vy + dy[k]
      (wx,wy,wz) = gluUnProject(x,y,z[k],mvmatrix,projmatrix,viewport)
      if self.debug :
        print(" at z = %f are (%f | %f | %f)" % (z[k],wx,wy,wz) )
      selectvol.append((wx,wy,wz))
      
    return selectvol
    
  def getSelectBodies(self,result):
    """
    Scan L{render.Render.renderPick} results; placing picked solid bodies
    and corresponding near/far distances in the SELECTBODIES array to be
    processed by the L{viewODE.grab} routine
    """
    figure_frame = self.renderer.figure.frame
    target_solid = self.renderer.target_solid
  
    if self.debug :
      print("Solids selected - shape : name at (near,far):")
      
    if result :
      i = 0
      while i < len(result):
        near = result[i].near
        far  = result[i].far
        k = 0
        while k < len(result[i].names):
          key = result[i].names[k]
          if  key < len(figure_frame.solids) : 
            shape = figure_frame.solids[key].shape
            name  = figure_frame.solids[key].label
            self.SELECTBODIES.append( \
              ( figure_frame.solids[key].body, \
                near,                          \
                far                            ) )
          elif key == 999 :
            shape = target_solid.shape
            name  = target_solid.label
            self.SELECTBODIES.append( \
              ( target_solid.body, near, far) )
          else :
            shape = "UNKNOWN"
            name  = "UNKNOWN"
          if self.debug : 
            print("  %s : %s at (%f | %f)" % (shape,name,near,far) )
          k = k + 1
        i = i + 1
      if len(self.SELECTBODIES) > 0 :
        self.pickClosestSelectBody()
        self.calcSelectNvecFvecCvec()
    else :
      if self.debug : print("- NOTHING!")

  def doMouseMotion(self, mx, my):
    """
    Mouse motion processing.
    """
    got = False
     
    if self.is_selecting :
      # Update location of selection volume
      got = True
      self.SELECTVXYZ = self.getSelectVolume(8,mx,my)
      if len(self.SELECTBODIES) == 0 :
        # Nothing selected yet; perform render picking
        result = self.glSelectWithCallback(mx, my, self.renderPick)
        self.getSelectBodies(result)
        glutPostRedisplay()

    return got
      
  def doMouseButton(self, b, s, mx, my):
    """
    Mouse button processing.
    """
    got = False
    
    if ( b == GLUT_LEFT_BUTTON and s == GLUT_DOWN ):
      got = True
      self.is_selecting = True
      self.SELECTVXYZ   = self.getSelectVolume(8,mx,my)
      self.SELECTBODIES = []
      result = self.glSelectWithCallback(mx, my, self.renderPick)
      self.getSelectBodies(result)
      glutPostRedisplay()
    elif ( b == GLUT_LEFT_BUTTON and s == GLUT_UP ):
      got = True
      self.is_selecting = False
      if self.debug : self.clearSelectBodies()
      else          : self.clearSelectVolume()
      glutPostRedisplay()
      
    return got
