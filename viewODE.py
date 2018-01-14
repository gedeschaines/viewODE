#!/usr/bin/python

# PyODE viewODE.py:  jointed ODE figure viewing and 
#                    manipulation

# Originally by Gary Deschaines

import sys, os, time
  
from math   import *
from string import *
   
try:
  import OpenGL
  OpenGL.USE_FREEGLUT = False
  from OpenGL.GL import *
  from OpenGL.GLU import *
  from OpenGL.GLUT import *
except:
  print("Error: PyOpenGL not installed properly !!")
  sys.exit()
  
try:
  import ode
except:
  print("Error: PyODE not installed properly !!")
  sys.exit()

sys.path.insert(1,os.getcwd())

try:
  from Figure import *
except:
  print("Error: Figure not installed properly !!")
try: 
  from Solids import *
except:
  print("Error: Solids not installed properly !!")
try:   
  from Render import *
except:
  print("Error: Render not installed properly !!")
try:
  from vecMath import *
except:
  print("Error: vecMath not installed properly !!")
     
def create_target_object(world, space, px, py, pz):
  """
  Create a target object at the specified position (px,py,pz) in the
  given ODE world space.
  
  Returns ((x,y,z),target) 
   
  @type  world: ODE world
  @param world: An ODE world returned by call to ode.World().
  @type  space: ODE space
  @param space: An ODE space returned by call to ode.Space().
  @type  px: number
  @param px: World space position x coordinate.
  @type  py: number
  @param py: World space position y coordinate.
  @type  pz: number
  @param pz: World space position z coordinate.
  
  @rtype:  tuple
  @return: ((x,y,z),target) -- position coordinates in world space for
           center of the target body and the target's ODE body object.
          
  """
  r       = 0.25
  x       = px
  y       = py 
  z       = pz
  density = 100
  solid   = solidBall(world, space, "target", density, r)
  target  = solid.body
  target.setPosition( (x, y, z) )
    
  return ((x, y, z), target)

def delete_target_object(target):
  """
  Delete target object created by the L{create_target_object} method.
      
  @type  target: ODE body object
  @param target: The ODE body object created by call to create_target_object.
  """  
  if target :
    target.disable()
    geom  = target.solid.geom
    space = geom.getSpace()
    space.remove(geom)
    target = None

def resetSim(mode):
  global RESET_MODE, INITIAL_POS, PAUSE
  global contactgroup
  global state
  global renderer
  global figure
  global target
  
  # Reset simulation state
  state = 0
  #PAUSE = True
  
  if mode == RESET_MODE['hard'] :
    # Reset figure configuration
    print(" ")
    print("**** Figure Configuration Reset ****")
    figure.resetConfig()
    # Reset rendering and capturing
    print(" ")
    print("**** Rendering and Capturing Reset ****")
    renderer.resetConfig()
    
  # Reset figure animation state
  if mode != RESET_MODE['exit'] :
    print(" ")
    print("**** Figure Animation Reset ****")
    figure.resetAnimation()

  # Empty contactgroup joints
  contactgroup.empty()
  
  # Delete figure frame elements
  figure.delete()
  
  # Delete target
  delete_target_object(target)
  
  if mode != RESET_MODE['exit'] :
    print(" ")
    print("**** Simulation Reset ****")
    printConfig()
  
def exitSim(reset_mode):
  resetSim(reset_mode)
  ode.CloseODE()
  sys.exit (0)
  
def printConfig():
  global renderer
  global figure
  
  print("---------- Rendering Configuration ----------")
  renderer.printConfig()  
  print("---- Figure Configuration ----")
  figure.printConfig()
  print("---------------------------------------------")

def _keyfunc (key, x, y):
  """
  Keypress handler.
  
  This method is passed to the Render class constructor in order to 
  be registered as the glutKeyboardFunc callback.
  
  @type  key: char
  @param key: Keypress code.
  @type  x: int
  @param x: Window x coordinate.
  @type  y: int
  @param y: Window y coordinate.
  """
  global RESET_MODE, PAUSE
  global renderer
  global figure
  global state
  
  key = key.decode()
  
  if key == '\x1b' :
    exitSim(RESET_MODE['exit'])
    return
   
  if figure.doKeyPress(key) :
    if figure.getAnimationState() == 0 :
      resetSim(RESET_MODE['soft'])
    return    

  if renderer.doKeyPress(key) :
    return
     
  if key == 'a' or key == 'A' :
  # Assemble object
    resetSim(RESET_MODE['soft'])
  elif key == 'g' or key == 'G' :
  # Toggle Render debug print
    renderer.toggleDebug()
  elif key == 'h' or key == 'H' :
  # Toggle Actions debug print
    figure.toggleActionsDebug()
  elif key == 'j' or key == 'J' :
  # Toggle Control debug print
    figure.toggleControlDebug()
  elif key == 'x' or key == 'X' :
  # Reset simulation
    resetSim(RESET_MODE['hard'])
  elif key == 'z' or key == 'Z' :
  # Togle simulation pause state
    PAUSE = not PAUSE
    if PAUSE : print("*** Simulation pause")
    else     : print("*** Simulation resume")
      
def _mousefunc(b, s, mx, my):
  """
  Mouse button handler.
  
  This method is passed to the Render class constructor in order to
  be registered as the glutMouseFunc callback.
  
  @type  b: number
  @param b: Mouse button code.
  @type  s: number
  @param s: Mouse button state.
  @type  mx: int
  @param mx: Mouse cursor position x coordinate.
  @type  my: int
  @param my: Mouse cursor position y coordinate.
  
  """
  global renderer
  global PAUSE
  
  if renderer.doMouseButton(b, s, mx, my) :
    # Left or right mouse button press/release
    return
    
  if b == GLUT_RIGHT_BUTTON and s == GLUT_DOWN :
    PAUSE = True
  elif b == GLUT_RIGHT_BUTTON and s == GLUT_UP :
    PAUSE = False
    
def grab(body, tstep):
  """
  Grabs a body solid.

  A lateral force is applied to the grabbed body solid in order to manipulate
  the body.
  
  @type  body: ODE body object
  @param body: The body returned by the Select pick processing.
  @type  tstep: number
  @param tstep: The simulation time step.
  """
  global world
  global renderer
  global figure
  global target
    
  if body :
      
    # Get picked body info 
    ( body, near, far ) = renderer.selector.getPickedBodyInfo()
    
    # Calculate selection volume near and far vectors
    renderer.selector.calcSelectNvecFvecCvec()
    
    # Get previous selection volume near and far vectors
    (prev_n_vec, prev_f_vec) = renderer.selector.getPrevNvecFvec()

    if prev_n_vec :
      
      # Get current selection volume near and far vectors
      (n_vec, f_vec) = renderer.selector.getSelectNvecFvec()
      
      # Calculate grab direction vector
      d_vec = vecSub(n_vec, prev_n_vec)
      d_mag = vecMag(d_vec)
      if d_mag > 0.01 : 
        d_vec = vecMulS(d_vec, 1.0/d_mag)
      else            :
        d_mag = 0.0
        d_vec = (0.0, 0.0, 0.0)
      
      # Apply grab force
      if body == target :
        if d_mag > 0.0 : 
          # grab force
          f     = body.mass*(2.0/tstep)
          f_vec = vecMulS(d_vec, f)
          body.addForce(f_vec)
      elif figure.frame.joints == [] :
        # Collapsed figure
        if d_mag > 0.0 : 
          # grab force
          f     = body.mass*(2.0/tstep) 
          f_vec = vecMulS(d_vec, f)
          body.addForce(f_vec)          
        else :      
          # levitation force
          (gx, gy, gz) = world.getGravity()
          body.addForce((gx, -body.mass*gy, gz))
      else :
        # Connected figure -- do not apply force to joint
        if not body.solid.joint :
          if d_mag > 0.0 :
            # grab force
            f     = figure.getTotMass()*(0.2/tstep)
            f_vec = vecMulS(d_vec, f)
            body.addForce(f_vec)
            
def near_callback(args, geom1, geom2):
  """ 
  Callback function for the collide() method.

  This function checks if the given geoms do collide and creates contact
  joints if they do.
  """
  global figure
  
  body1, body2 = geom1.getBody(), geom2.getBody()
  if (body1 is None):
      body1 = ode.environment
  if (body2 is None):
      body2 = ode.environment
  if (body1 == body2):
      return
  if (ode.areConnected(body1, body2)):
      return
      
  # Check if the objects do collide
  contacts = ode.collide(geom1, geom2)

  # Create contact joints
  world, contactgroup = args
  for c in contacts:
    (pos,nrm,depth,g1,g2) = c.getContactGeomParams()
    if isinstance(g2,ode.GeomPlane) :
      if g1.solid.label == "R_foot" or \
         g1.solid.label == "R_ball" or \
         g1.solid.label == "R_toes" :
        c.setBounce(0.01)
        c.setMu(15000.0)
        figure.appendCGP('R',(pos,nrm))
      elif g1.solid.label == "L_foot" or \
           g1.solid.label == "L_ball" or \
           g1.solid.label == "L_toes" :
        c.setBounce(0.01)
        c.setMu(15000.0)
        figure.appendCGP('L',(pos,nrm))
      else :
        c.setBounce(0.2)
        c.setMu(1500.0)
    else :
      flags = ode.ContactSoftCFM
      c.setMode(flags)
      c.setSoftCFM(0.2E-2)
      c.setBounce(0.2)
      c.setMu(1500.0)
    j = ode.ContactJoint(world, contactgroup, c)
    j.attach(body1, body2)
        
def _idlefunc():
  """ 
  ODE simulation loop.
  
  This method is passed to the Render class constructor in order to be
  registered as the glutIdleFunc callback.
  """
  global PAUSE
  global renderer, frame_rate, frame_step
  global world, space, contactgroup
  global figure, target
  global state, lasttime
  global t, t_msec, dt, dt_msec, nstep, tstep, tstep_msec

  t_wait = dt - (time.time() - lasttime)
  if (t_wait > 0):
    time.sleep(t_wait)

  if not PAUSE :
    
    # State 0: Create ODE figure and target 
    if state == 0 :
      figure.createFrame(0.0, 0.0, -2.0)
      (tgtpos, target) = create_target_object( world, \
                                               space, \
                                               0.0, 3.5, 2.0 )
      target.setGravityMode(0)
      renderer.setRenderFigure(figure)
      renderer.setRenderTarget(target)
      if renderer.captor :
        # Zero capture millisecond counter
        renderer.captor.zeroMsecCounter()
      
      # Transition to animation state
      state  = 1
      t_msec = 0

    # Simulate object dynamics
    for n in range(nstep):
          
      if state == 1 :
        # Animation state -- check for picked body
        body = renderer.selector.getPickedBody()
        if body :
          # Update grab forces
          grab(body, tstep)
        # Update figure animation
        figure.updateAnimation(body, target, t, tstep)
      
      # Detect collisions and create contact joints
      figure.createCGP('R')
      figure.createCGP('L')
      space.collide((world,contactgroup), near_callback)
      figure.setCOP('R')
      figure.setCOP('L')
      
      # Simulation step
      world.step(tstep)
      #figure.calcZMP()

      # Remove all contact joints
      contactgroup.empty()
      
      # Update simulation time
      t      += tstep
      t_msec += tstep_msec
 
    if renderer.captor :
      # Update capture millisecond counter
      renderer.captor.updateMsecCounter(dt_msec)
        
    # Update display
    glutPostRedisplay()
    
  lasttime = time.time()
  
if __name__ == '__main__':
  
  # Simulation reset mode:
  #   soft -- resets figure to current config/states
  #   hard -- resets figure, rendering and capturing
  #           to initial config/states
  #   exit -- exits simulation  
  RESET_MODE = {'soft' : 0, 'hard' : 1, 'exit' : 2}

  # ODE simulation loop timing, counters and state
  PAUSE      = True
  fps        = 100
  t          = 0.0
  t_msec     = 0
  dt         = 1.0/float(fps)
  dt_msec    = int(dt*1000.0)
  nstep      = 4
  tstep      = dt/float(nstep)
  tstep_msec = int(tstep*1000.0)
  state      = 0

  # Capture parameters
  frame_rate = 25
  frame_time = int(1000.0/float(frame_rate))  # in milliseconds rounded
  frame_time = dt_msec*(frame_time/dt_msec)   # to multiple of dt_msec
  filename   = "image"

  # Initialize OpenGL renderer-selector and PIL captor
  x        = 50
  y        = 50
  width    = 800
  height   = 600
  renderer = Render(x, y,          \
                    width, height, \
                    _idlefunc,     \
                    _keyfunc,      \
                    _mousefunc,    \
                    frame_time,    \
                    filename       )

  # Create ODE world object
  world = ode.World()
  world.setGravity( (0.0,-9.81,0.0) )
  world.setERP(0.2)
  world.setCFM(0.2E-3)
  
  # Create ODE space object
  space = ode.Space()

  # Create a plane geom which prevent the objects
  # from falling forever
  floor = ode.GeomPlane(space, (0.0,1.0,0.0), 0)

  # A ODE joint group for the contact joints that are
  # generated whenever two bodies collide
  contactgroup = ode.JointGroup()

  # Globals for ODE figure and target body information
  sfac   = 1.0
  figure = Figure(world, space, floor, sfac)
  target = None
  
  # Print rendering, physical and dynamics configurations
  printConfig()
  
  if PAUSE :
    print("Press 'Z' key to begin simulation.")
    
  lasttime = time.time()
  
  # Begin simulation
  glutMainLoop()
