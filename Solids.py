# Solids.py
#
# viewODE solids definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Attributions
#
# + Matthias Baas and Pierre Gay for the Python-ODE Bindings examples program
#   tutorial3.py available at https://sourceforge.net/projects/pyode/ which was
#   used as a basis for the solid models.
#
# Disclaimers
#
#   See the file DISCLAIMER-GaryDeschaines
#   See the file DISCLAIMER-PyODE

from sys  import exit
from math import pi, sqrt

#
# Import ODE module for world, space, body, geom and mass models.

try:
  import ode
except:
  print("Error: This module requires PyODE !!")
  exit()

class Solids:
    
  def __init__(self, label):
    """
    A viewODE Solids class constructor to instantiate a viewODE world space
    solid object comprised of an ODE body and geom, and which may be attached
    to other solids with ODE joints and motors.
                    
    @param label: The solid's unique identifying label.
    @type  label: string
    """

    self.label   = label
    self.body    = None
    self.geom    = None
    self.joint   = None
    self.motor   = None
    self.mass    = 0.0
    self.Ixyz    = (0.0, 0.0, 0.0)
    self.wire    = 0
    self.shape   = ""
    self.boxsize = (0.0, 0.0, 0.0)
    self.radius  = 0.0
    self.length  = 0.0
    self.r1      = 0.0
    self.r2      = 0.0
    self.h       = 0.0
    self.t0      = 0.0         # cm initialization time
    self.knt     = 0           # cm update counter
    self.cmpos   = [[],[],[]]  # cm position vector circular buffer
    self.cmvel   = []          # cm velocity vector
    self.cmacc   = []          # cm acceleration vector
    
def solidLabel(side, name):
  """
  Generate a solid label from side and name.
  """
  if side and name : label = str.join('',(side, '_', name))
  else             : label = name
    
  return label
  
def solidHeight(solid):
  """
  Get a solid's height as measured in +Y direction.
  """
  if solid.shape == "sphere" :
    height = solid.radius*2.0
  elif solid.shape == "cone" :
    height = solid.h
  elif solid.shape == "rod"  :
    height = solid.length
  elif solid.shape == "box"  :
    (lx, height, lz) = solid.boxsize
  else:
    print("unknown solid.shape %s" % solid.shape)
    exit(0)
    
  return height
  
def solidIxyz(solid):
  """
  Calculate and return the given solid's moments of inertia about
  its principal axes.

  @param solid: A solid.
  @type  solid: viewODE Solids object

  @return: Principle moments of inertia (Ixx,Iyy,Izz).
  @rtype: tuple
  """
  if solid.shape == "sphere" :
    m   = solid.mass
    r   = solid.radius
    Ixx = (2.0/5.0)*m*r*r
    Iyy = Ixx
    Izz = Ixx
  elif solid.shape == "cone" :
    m    = solid.mass
    r    = solid.r1
    h    = solid.h
    rsq  = r*r
    hsq  = h*h
    mrsq = m*rsq
    Ixx  = (3.0/80.0)*(4.0*mrsq + m*hsq)  # Approximate for truncated
    Iyy  = (3.0/10.0)*mrsq                # right circular cone
    Izz  = Ixx
  elif solid.shape == "rod"  :
    m    = solid.mass
    r    = solid.radius
    h    = solid.length
    rsq  = r*r
    hsq  = h*h
    mrsq = m*rsq
    Ixx  = mrsq/4.0 + m*hsq/12.0
    Iyy  = mrsq/2.0
    Izz  = Ixx
  elif solid.shape == "box"  :
    m       = solid.mass
    (a,b,c) = solid.boxsize
    asq     = a*a
    bsq     = b*b
    csq     = c*c
    mdiv12  = m/12.0
    Ixx     = mdiv12*(bsq + csq)
    Iyy     = mdiv12*(asq + csq)
    Izz     = mdiv12*(asq + bsq)
  else:
    print("unknown solid.shape %s" % solid.shape)
    exit(0)

  return (Ixx, Iyy, Izz)
  
def solidBox(world, space, label, density, lx, ly, lz):
  """
  Create a box body and its corresponding geom.

  @param world: The viewODE world.
  @type  world: ODE World object
  @param space: The viewODE world space.
  @type  space: ODE Space object
  @param label: The solid's label.
  @type  label: string
  @param density: The solid's material density.
  @type  density: float
  @param lx: Length of box x side.
  @type  lx: float
  @param ly: Length of box y side.
  @type  ly: float
  @param lz: Length of box z side.
  @type  lz: float

  @return: The created box object.
  @rtype: viewODE Solids object
  """
  # Create body
  body = ode.Body(world)
  M    = ode.Mass()
  M.setBox(density, lx, ly, lz)
  body.setMass(M)
  body.mass = density*lx*ly*lz
    
  # Create a box geom for collision detection
  geom = ode.GeomBox(space, lengths=(lx, ly, lz))
  geom.setBody(body)
    
  # Set parameters for dynamics and rendering
  solid         = Solids(label)
  solid.body    = body
  solid.geom    = geom
  solid.mass    = body.mass
  solid.wire    = 0
  solid.shape   = "box"
  solid.boxsize = (lx, ly, lz)
  solid.Ixyz    = solidIxyz(solid)
    
  # ODE object links to solid object
  body.solid = solid
  geom.solid = solid
    
  return solid
      
def solidBall(world, space, label, density, r):
  """
  Create a ball body and its corresponding geom.

  @param world: The viewODE world.
  @type  world: ODE World object
  @param space: The viewODE world space.
  @type  space: ODE Space object
  @param label: The solid's label.
  @type  label: string
  @param density: The solid's material density.
  @type  density: float
  @param r: Radius of ball.
  @type  r: float

  @return: The created ball object.
  @rtype: viewODE Solids object
  """
  # Create body
  body = ode.Body(world)
  M    = ode.Mass()
  M.setSphere(density, r)
  body.setMass(M)
  body.mass = density*(4.0/3.0)*pi*r*r*r
    
  # Create a sphere geom for collision detection
  geom = ode.GeomSphere(space, radius=r)
  geom.setBody(body)
    
  # Set parameters for dynamics and rendering
  solid        = Solids(label)
  solid.body   = body
  solid.geom   = geom
  solid.mass   = body.mass
  solid.wire   = 0
  solid.shape  = "sphere"
  solid.radius = r
  solid.Ixyz   = solidIxyz(solid)
  
  # ODE object links to solid object
  body.solid = solid
  geom.solid = solid
    
  return solid
        
def solidRod(world, space, label, density, r, l):
  """
  Create a vertical cylindrical rod body and its corresponding geom.

  @param world: The viewODE world.
  @type  world: ODE World object
  @param space: The viewODE world space.
  @type  space: ODE Space object
  @param label: The solid's label.
  @type  label: string
  @param density: The solid's material density.
  @type  density: float
  @param r: Radius of rod.
  @type  r: float
  @param l: Length of rod.
  @type  l: float

  @return: The created rod object.
  @rtype: viewODE Solids object
  """
  # Create body
  body = ode.Body(world)
  M    = ode.Mass()
  M.setCylinder(density, 2, r, l)
  body.setMass(M)
  body.mass = density*pi*r*r*l
    
  # Create a cylinder geom for collision detection
  # geom = ode.GeomCylinder(space, radius=r, length=l)
  # geom.setBody(body)
  '''
  geom.setRotation([1., 0., 0., 0., 0., 1., 0., -1., 0.])
  R = geom.getRotation()
  print("%s: Rotation Matrix\n" \
    "\t(%8.3f | %8.3f | %8.3f)\n" \
    "\t(%8.3f | %8.3f | %8.3f)\n" \
    "\t(%8.3f | %8.3f | %8.3f)" % \
    (label, R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]) )
  AABB = geom.getAABB()
  print("%s: AABB\n" \
    "\t(%8.3f | %8.3f) (%8.3f | %8.3f) (%8.3f | %8.3f)" % \
    (label, AABB[0], AABB[1], AABB[2], AABB[3], AABB[4], AABB[5]) )
  '''
  # Create an inscribed box geom for collision detection
  geom = ode.GeomBox(space, lengths=(sqrt(2)*r, l, sqrt(2)*r))
  geom.setBody(body)
    
  # Set parameters for dynamics and rendering
  solid        = Solids(label)
  solid.body   = body
  solid.geom   = geom
  solid.mass   = body.mass
  solid.wire   = 0
  solid.shape  = "rod"
  solid.radius = r
  solid.length = l
  solid.Ixyz   = solidIxyz(solid)
  
  # ODE object links to solid object
  body.solid = solid
  geom.solid = solid
    
  return solid
    
def solidCone(world, space, label, density, r1, r2, h):
  """
  Create a vertical cylindrical cone body and its corresponding geom.

  @param world: The viewODE world.
  @type  world: ODE World object
  @param space: The viewODE world space.
  @type  space: ODE Space object
  @param label: The solid's label.
  @type  label: string
  @param density: The solid's material density.
  @type  density: float
  @param r1: Radius of cone base.
  @type  r1: float
  @param r2: Radius of cone top.
  @type  r2: float
  @param h: Height of cone.
  @type  h: float

  @return: The created cone object.
  @rtype: viewODE Solids object
  """
  # Create body
  body = ode.Body(world)
  M    = ode.Mass()
  M.setCylinder(density, 2, r1, h)  # an approximation
  body.setMass(M)
  body.mass = density*(1.0/3.0)*pi*h*(r1*r1+r1*r2+r2*r2)  # exact
        
  # Create a cylinder geom for collision detection
  # geom = ode.GeomCylinder(space, radius=r1, length=h)
  # geom.setBody(body)
    
  # Create a circumscribed box geom for collision detection
  geom = ode.GeomBox(space, lengths=(2*r1, h, 2*r1))
  geom.setBody(body)
    
  # Set parameters for dynamics and rendering
  solid       = Solids(label)
  solid.body  = body
  solid.geom  = geom
  solid.mass  = body.mass
  solid.wire  = 0
  solid.shape = "cone"
  solid.r1    = r1
  solid.r2    = r2
  solid.h     = h
  solid.Ixyz  = solidIxyz(solid)
        
  # ODE object links to solid object
  body.solid = solid
  geom.solid = solid
  
  return solid
