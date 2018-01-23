# Render.py
#
# viewODE rendering definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Attributions
#
# + Matthias Baas and Pierre Gay for the Python-ODE Bindings examples program
#   tutorial3.py available at https://sourceforge.net/projects/pyode/ which was
#   used as a basis for solids rendering.
#
# Disclaimers
#
#   See the file DISCLAIMER-GaryDeschaines
#   See the file DISCLAIMER-PyODE

from sys import argv, exit

#
# Import OpenGL modules for rendering and ODE module for body models.

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
  import ode
except:
  print("Error: This module requires PyODE !!")
  exit()

#
# Import viewODE modules for rendered solids selection and image capture.

try:
  from Select  import *
except:
  print("Error: Select not installed properly !!")  
try:
  from Capture import *
except:
  print("Error: Capture not installed properly !!")
    
class Render:
    
  def __init__(self,
               x, y,
               width, height,
               idlefunc,
               keyboardfunc,
               mousefunc,
               frame_time,
               filename ):
    """
    Render Constructor -- ODE figure solid rendering.
                    
    Initialize the ODE figure solid renderer.
    """
    
    # Initialize rendering parameters
    
    self.debug        = False
    self.figure       = None
    self.target_solid = None

    # Viewing volume parameters
    
    self.projmode = 1
    self.x        = x
    self.y        = y
    self.width    = width
    self.height   = height
    self.near     = 10
    self.far      = 30
    self.eye      = ( [14.0, 2.5, 14.0],
                      [0.0,  2.5,  0.0],
                      [0.0,  1.0,  0.0] )
                      
    # Modeling transform parameters
    
    self.rotate    = False
    self.last_x    = 0
    self.last_y    = 0
    self.rotationX = 0.0
    self.rotationY = 0.0
    
    # Instantiate selector
    
    self.selector = Select(self, self.renderPick)
    
    # Instantiate image captor
    
    self.captor        = None
    self.image_capture = False
    
    if filename != "" :
      self.captor = Capture(self, frame_time, filename)
     
    # Initialize Glut
     
    glutInit(argv)

    # Open a display window
    
    glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
    glutInitWindowPosition (self.x, self.y)
    glutInitWindowSize (self.width, self.height)
    glutCreateWindow ("viewODE".encode())
    self.prepare_GL()
    
    # Register GLUT callbacks
    
    self.idlefunc     = idlefunc
    self.keyboardfunc = keyboardfunc
    self.mousefunc    = mousefunc
    self.reshapefunc  = self.doReshape
    self.displayfunc  = self.doDisplay
    self.motionfunc   = self.doMouseMotion
    
    glutIdleFunc (self.idlefunc)
    glutReshapeFunc (self.reshapefunc)
    glutDisplayFunc (self.displayfunc)
    glutKeyboardFunc (self.keyboardfunc)
    glutMouseFunc (self.mousefunc)
    glutMotionFunc (self.motionfunc)
    
  def debugOn(self)    : self.debug = True
  def debugOff(self)   : self.debug = False
  def toggleDebug(self) :
    self.debug = not self.debug
    self.selector.toggleDebug()
    
  def prepare_GL(self):
    """
    Prepare GL drawing.
    """
    # Initialize
    glClearColor(0.9,0.9,0.9,1.0)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_NORMALIZE)  # Prevents single color plane surfaces
    glShadeModel(GL_FLAT)   # Keep faceted look
    self.setBasicLighting()
    glMaterialf(GL_FRONT, GL_SHININESS, 80.0)
    
  def doReshape(self, w, h):
    """
    Window reshape callback
    """ 
    glViewport( 0, 0, w, h )
    self.setProjection(w,h)
    glutPostRedisplay()

  def setBasicLighting(self):
    """
    Set basic white diffuse, specular and ambient lighting.
    """
    light_position = [-5.0,10.0,10.0, 0.0]
    white_light    = [ 1.0, 1.0, 1.0, 1.0]
    lmodel_ambient = [ 0.2, 0.2, 0.2, 1.0]
    glLightfv(GL_LIGHT0, GL_POSITION, light_position)
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light)
    glLightfv(GL_LIGHT0, GL_SPECULAR, white_light)
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    
  def setProjection(self, w, h):
    """
    Set view volume projection parameters.
    """
    self.width  = w
    self.height = h
    xwid = 4.0
    xmax = xwid/2.0
    xmin = -xmax
    ywid = 4.0
    ymax = ywid/2.0
    ymin = -ymax
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    if w > h :
      sfac = float(w)/float(h)
      if self.projmode : 
        glFrustum(xmin*sfac, xmax*sfac, ymin, ymax, self.near, self.far)
      else :
        xmin = xmin*2.0
        xmax = xmax*2.0
        ymin = ymin*2.0
        ymax = ymax*2.0
        glOrtho(xmin*sfac, xmax*sfac, ymin, ymax, self.near, self.far)
    else :
      sfac = float(h)/float(w)
      if self.projmode : 
        glFrustum(xmin, xmax, ymin*sfac, ymax*sfac, self.near, self.far)
      else : 
        xmin = xmin*2.0
        xmax = xmax*2.0
        ymin = ymin*2.0
        ymax = ymax*2.0
        glOrtho(xmin, xmax, ymin*sfac, ymax*sfac, self.near, self.far)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt( self.eye[0][0], self.eye[0][1], self.eye[0][2],
               self.eye[1][0], self.eye[1][1], self.eye[1][2],
               self.eye[2][0], self.eye[2][1], self.eye[2][2] )
               
  def setRenderFigure(self, figure):
    
    self.figure = figure
    
  def setRenderTarget(self, target):
    
    self.target_solid = target.solid
    
  def getRenderTarget(self):
    
    return ( self.target_solid )

  def doDisplay (self):
    """
    Display callback
    """
    # Clear pixel color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
  
    # Draw the ODE geom floor plane
    self.renderFloor()
    
    # Draw selection volume
    if self.debug :
      self.selector.drawSelectVolume()
  
    # Draw figure frame solids
    if self.figure :
      self.renderFigure(self.figure, GL_RENDER)
      
    # Draw target body solid
    if self.target_solid :
      self.renderSolid(self.target_solid, 1)
      
    # Capture image drawn in back buffer (at specified frame rate)
    if self.captor and self.image_capture :
      self.captor.saveImage()

    glutSwapBuffers()
    
  def drawWorldAxes(self):
    """
    Draws labeled world space reference system axes.
    """
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f( 0.0, 0.0, 0.0)
    glVertex3f( 0.8, 0.0, 0.0)  # X-axis
    glEnd()
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f( 0.0, 0.0, 0.0)
    glVertex3f( 0.0, 0.8, 0.0)  # Y-axis
    glEnd()
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINES)
    glVertex3f( 0.0, 0.0, 0.0)
    glVertex3f( 0.0, 0.0, 0.8)  # Z-axis
    glEnd()
    glColor3f(1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f( 1.0, 0.1, 0.1)  # letter X
    glVertex3f( 1.0,-0.1,-0.1)
    glVertex3f( 1.0, 0.1,-0.1)
    glVertex3f( 1.0,-0.1, 0.1)
    glEnd()
    glColor3f(0.0, 1.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f( 0.0, 1.1, 0.0)  # letter Y
    glVertex3f(-0.1, 1.2, 0.0)
    glVertex3f( 0.0, 1.1, 0.0)
    glVertex3f( 0.1, 1.2, 0.0)
    glVertex3f( 0.0, 1.1, 0.0)
    glVertex3f( 0.0, 1.0, 0.0)
    glEnd() 
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_LINE_STRIP)
    glVertex3f(-0.1, 0.1, 1.0)  # letter Z
    glVertex3f( 0.1, 0.1, 1.0)
    glVertex3f(-0.1,-0.1, 1.0)
    glVertex3f( 0.1,-0.1, 1.0)
    glEnd()
    
  def renderFloor(self):
    """
    Draw an ODE floor plane grid.
    """
    glDisable(GL_LIGHTING)   # Disable lighting in order for
    glColor3f(0.0, 0.0, 0.0) # lines to always be this color
    glPushMatrix()
    glRotatef(self.rotationY, 0.0, 1.0, 0.0)
    glRotatef(self.rotationX, 1.0, 0.0, 0.0)
    for j in range(8):
      z = 4.0 - j
      for i in range(8):
        x = -4.0 + i
        glBegin(GL_LINE_LOOP)
        glVertex3f(x      , 0.0, z      )
        glVertex3f(x      , 0.0, z - 1.0)
        glVertex3f(x + 1.0, 0.0, z - 1.0)
        glVertex3f(x + 1.0, 0.0, z      )
        glEnd()
    self.drawWorldAxes()
    glPopMatrix()
    glEnable(GL_LIGHTING)
    
  def drawFigureCoM(self, figure):
    """
    Draw figure's CoM on the floor plane as a red diamond.
    """   
    com = figure.calcCenterOfMass()
    glDisable(GL_LIGHTING)   # Disable lighting in order for
    glColor3f(1.0, 0.0, 0.0) # lines to always be this color
    glPushMatrix()
    glRotatef(self.rotationY, 0.0, 1.0, 0.0)
    glRotatef(self.rotationX, 1.0, 0.0, 0.0)
    glBegin(GL_LINE_LOOP)
    glVertex3f(com[0]+0.05, 0.0, com[2]     )
    glVertex3f(com[0]     , 0.0, com[2]+0.05)
    glVertex3f(com[0]-0.05, 0.0, com[2]     )
    glVertex3f(com[0]     , 0.0, com[2]-0.05)
    glEnd()
    glPopMatrix()
    glEnable(GL_LIGHTING)
    
  def drawFigureCoP(self, figure, foot):
    """
    Draw figure's CoP on the floor plane as a vertical purple line.
    """   
    cop = figure.getCOP(foot)
    glDisable(GL_LIGHTING)   # Disable lighting in order for
    glColor3f(1.0, 0.0, 1.0) # lines to always be this color
    glPushMatrix()
    glRotatef(self.rotationY, 0.0, 1.0, 0.0)
    glRotatef(self.rotationX, 1.0, 0.0, 0.0)
    glBegin(GL_LINES)
    glVertex3f(cop[0],-0.2,cop[2])
    glVertex3f(cop[0], 0.2,cop[2])
    glEnd()
    glPopMatrix()
    glEnable(GL_LIGHTING)
    
  def drawFigureZMP(self, figure):
    """
    Draw figure's ZMP on the floor plane as a blue square.
    """   
    zmp = figure.getZMP()
    glDisable(GL_LIGHTING)   # Disable lighting in order for
    glColor3f(0.0, 0.0, 1.0) # lines to always be this color
    glPushMatrix()
    glRotatef(self.rotationY, 0.0, 1.0, 0.0)
    glRotatef(self.rotationX, 1.0, 0.0, 0.0)
    glBegin(GL_LINE_LOOP)
    glVertex3f(zmp[0]+0.05, 0.0, zmp[2]+0.05)
    glVertex3f(zmp[0]-0.05, 0.0, zmp[2]+0.05)
    glVertex3f(zmp[0]-0.05, 0.0, zmp[2]-0.05)
    glVertex3f(zmp[0]+0.05, 0.0, zmp[2]-0.05)
    glEnd()
    glPopMatrix()
    glEnable(GL_LIGHTING)
    
  def renderFigure(self, figure, mode):
    
    if mode == GL_SELECT :
      if figure.frame :
        for k in range(len(figure.frame.solids)):
          glPushName(k)
          self.renderSolid(figure.frame.solids[k], 0)
          glPopName()
      return
        
    if mode == GL_RENDER :
      if figure.frame :
        self.drawFigureCoM(figure)
        self.drawFigureZMP(figure)
        self.drawFigureCoP(figure,'R')
        self.drawFigureCoP(figure,'L')
        for s in figure.frame.solids:
          self.renderSolid(s, 1)
      return
  
  def drawBodyAxes(self, solid):
    """
    Draws solid's body frame axes.
    """
    dxy = solid.radius * 2.0
    dz  = dxy
    glColor3f(1.0, 0.0, 0.0)
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [1.0, 0.0, 0.0, 1.0])
    glBegin(GL_LINES)
    glVertex3f( 0.0, 0.0, 0.0)
    glVertex3f( dxy, 0.0, 0.0)  # x-axis
    glEnd()
    glColor3f(0.0, 1.0, 0.0)
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0.0, 1.0, 0.0, 1.0])
    glBegin(GL_LINES)
    glVertex3f( 0.0, 0.0, 0.0)
    glVertex3f( 0.0, dxy, 0.0)  # y-axis
    glEnd()
    glColor3f(0.0, 0.0, 1.0)
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0.0, 0.0, 1.0, 1.0])
    glBegin(GL_LINES)
    glVertex3f( 0.0, 0.0, 0.0)
    glVertex3f( 0.0, 0.0, dz )  # z-axis
    glEnd()
    glColor3f(0.0, 0.0, 0.0)
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0.0, 0.0, 0.0, 1.0])
  
  def renderSolid(self, solid, mode):
    """
    Draw an ODE body in OpenGL.
    """
    
    body  = solid.body
    x,y,z = body.getPosition()
    R     = body.getRotation()
    rot   = [R[0], R[3], R[6], 0.,
             R[1], R[4], R[7], 0.,
             R[2], R[5], R[8], 0.,
             x, y, z, 1.0]
         
    glPushMatrix()
    glRotatef(self.rotationY, 0.0, 1.0, 0.0)
    glRotatef(self.rotationX, 1.0, 0.0, 0.0)
    
    if solid.shape == "box" :
      sx,sy,sz = solid.boxsize
      glMultMatrixd(rot)
      glScalef(sx, sy, sz)
      if solid.wire and mode: glutWireCube(1)
      else                  : glutSolidCube(1)
    elif solid.shape == "sphere" :
      r = solid.radius
      glMultMatrixd(rot)
      self.drawBodyAxes(solid)
      glRotatef(90.0, 0.0, 1.0, 0.0)  # Note: align GLU sphere +z-axis 
                                      # with the ODE world +x-axis
      if solid.wire and mode : glutWireSphere(r,9,9)
      else                   : glutSolidSphere(r,9,9)
    elif solid.shape == "rod" :
      r = solid.radius
      l = solid.length 
      glMultMatrixd(rot)
      glTranslatef(0.0, -l/2, 0.0)  # diff between ODE and GLU body origin
      glRotatef(-90.0, 1.0, 0.0, 0.0)  # Note: align GLU cylinder +z-axis 
                                       # with the ODE cylinder +y-axis
      quad = gluNewQuadric()
      if solid.wire and mode : gluQuadricDrawStyle(quad,GLU_LINE)
      else                   : gluQuadricDrawStyle(quad,GLU_FILL)
      gluCylinder(quad,r,r,l,9,1)
      gluDeleteQuadric(quad)
    elif solid.shape == "cone" :
      r1 = solid.r1
      r2 = solid.r2
      h  = solid.h
      glMultMatrixd(rot)
      glTranslatef(0.0, -h/2, 0.0)  # diff between ODE and GLU body origin
      glRotatef(-90.0, 1.0, 0.0, 0.0)  # Note: align GLU cylinder +z-axis 
                                        # with the ODE world +y-axis 
      quad = gluNewQuadric()
      if solid.wire and mode : gluQuadricDrawStyle(quad,GLU_LINE)
      else                   : gluQuadricDrawStyle(quad,GLU_FILL)
      gluCylinder(quad,r1,r2,h,9,1)
      gluDeleteQuadric(quad)

    glPopMatrix()
    
  def renderPick(self):
    """
    Render pick process passed as callback to L{select.Select.glSelectWithCallback}.
    """
    if self.figure :
      self.renderFigure(self.figure, GL_SELECT)
                    
    if self.target_solid :
      glPushName(999)
      self.renderSolid(self.target_solid, 0)
      glPopName()
      
  def doMouseButton(self, b, s, mx, my):
    """
    Mouse button processing.
    """
    if self.figure == None : return None
    
    if self.selector.doMouseButton(b, s, mx, my) :
      # Left mouse button pressed or released
      return True
      
    got = False
    
    if b == GLUT_MIDDLE_BUTTON and s == GLUT_DOWN :
      got = True
      self.rotate = True
      self.last_x = mx
      self.last_y = my
    elif b == GLUT_MIDDLE_BUTTON and s == GLUT_UP :
      got = True
      self.rotate = False
    
    return got
      
  def doMouseMotion(self, mx, my):
    """
    Mouse motion processing.
    """
    if self.selector.isSelecting() :
      # Left mouse button still pressed
      self.selector.doMouseMotion(mx, my)
      return True
      
    if self.rotate == True :
      # Middle mouse button still pressed
      self.rotationX -= float(my - self.last_y)
      self.rotationY += float(mx - self.last_x)
      self.last_x = mx
      self.last_y = my
      glutPostRedisplay()
      return True
      
    return False
    
  def resetConfig(self):
  
    self.projmode  = 1
    self.rotationX = 0.0
    self.rotationY = 0.0
    
    self.selector.clearSelectVolume()
    
    if self.captor :
      self.image_capture = False
      
    glutPostRedisplay()
          
  def printConfig(self):
    
    if self.captor :
      if self.image_capture : print("Image Capture ON")
      else                  : print("Image Capture OFF")
    else :
      print("Image Capture Not Enabled")
      
    if self.projmode : print("Perspective Projection")
    else             : print("Orthographic Projection")
      
  def doKeyPress(self, key):
    
    got = False
  
    if key == 'i' or key == 'I' :
      # Toggle image capture
      got = True
      if self.captor :
        self.image_capture = not self.image_capture
        if self.image_capture :
          self.captor.zeroMsecCounter()
          print("Image Capture ON")
        else : 
          print("Image Capture OFF")
      else :
        print("Image Capture Not Enabled")
    elif key == 'o' or key == 'O' :
      # Orthographic projection
      got = True
      if self.projmode == 0 : return
      self.projmode = 0
      self.setProjection(self.width,self.height)
      self.selector.clearSelectVolume()
      glutPostRedisplay()
    elif key == 'p' or key == 'P' :
      # Perspective projection
      got = True
      if self.projmode == 1 : return
      self.projmode = 1
      self.setProjection(self.width,self.height)
      self.selector.clearSelectVolume()
      glutPostRedisplay()
    elif key == '.' :
      # Reset rotation angles
      got = True
      self.rotationX = 0.0
      self.rotationY = 0.0
      glutPostRedisplay()
    elif key == 'v' or key == 'V' :
      # Clear selection volume
      got = True
      self.selector.clearSelectVolume()
      glutPostRedisplay()

    return got
