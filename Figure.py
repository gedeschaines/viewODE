# Figure.py
#
# viewODE humanoid jointed rigid body figure definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Attributions
#
# + Matt Heinzen for PyODE Ragdoll Physics Tutorial program ragdoll-pyode-tutorial.py
#   available at http://monsterden.net/software/ragdoll-pyode-tutorial which was used
#   as a basis for the humanoid figure herein.
#
# Disclaimers
#
#   See the file DISCLAIMER-GaryDeschaines
#   See the file DISCLAIMER-MatHeinzen

from sys import exit
#from math import isinf, isnan, sqrt  # imported within vecMath module

#
# Import ODE module for world, space, body and joint models.

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
# Import viewODE modules for frame, control, action and vector math.

try:
  from Frame import *
except:
  print("Error: Frame not installed properly !!")
try:
  from Control import *
except:
  print("Error: Control not installed properly !!")
try:
  from Actions import *
except:
  print("Error: Actions not installed properly !!")
try:
  from vecMath import *
except:
  print("Error: vecMath not installed properly !!")

ANIMATION_STATES = ( "Initialize",
                     "Stand Still",
                     "Stand Fixed",
                     "Suspended",
                     "Joint Manipulation",
                     "Solid Manipulation",
                     "React",
                     "Action",
                     "Resting",
                     "Standup" )

class Figure:

  def __init__(self, world, space, floor, sfac):
    """
    A viewODE Figure class constructor to instantiate a humanoid robotic
    figure object.

    @param world: The simulated dynamic world.
    @type  world: ODE World object
    @param space: The dynamic world space .
    @type  space: ODE Space object
    @param floor: The dynamic world space ground plane.
    @type  floor: ODE GeomPlane object
    @param sfac: The figure size scaling factor.
    @param sfac: float
    """
    self.world      = world
    self.space      = space
    self.floor      = floor
    self.sfac       = sfac
    self.origin     = (0.0, 0.0, 0.0)
    self.refpos     = (0.0, 0.0, 0.0)
    self.frame      = Frame(self)
    self.control    = Control(self)
    self.actions    = Actions(self)

    # Dynamic figure states

    self.cgp_r_foot = []
    self.cop_r_foot = (0.0, 0.0, 0.0)
    self.cgp_l_foot = []
    self.cop_l_foot = (0.0, 0.0, 0.0)
    self.zmp        = (0.0, 0.0, 0.0)

    # Animation States
    
    self.INITIALIZE  = 0
    self.STANDSTILL  = 1
    self.STANDFIXED  = 2
    self.SUSPENDED   = 3
    self.JOINT_MANIP = 4
    self.SOLID_MANIP = 5
    self.REACT       = 6
    self.ACTION      = 7
    self.RESTING     = 8
    self.STANDUP     = 9

    # Animation State Parameters

    self.state      = 0
    self.last_state = None
    self.body       = None  # grabbed body
    self.target     = None  # rendered target
    self.t          = 0.0
    self.tstep      = 0.0

    # Animation State Methods

    self.AnimationMethods = {self.INITIALIZE: self.animationInitialize,
                             self.STANDSTILL: self.animationStandStill,
                             self.STANDFIXED: self.animationStandFixed,
                             self.SUSPENDED: self.animationSuspended,
                             self.JOINT_MANIP: self.animationJointManip,
                             self.SOLID_MANIP: self.animationSolidManip,
                             self.REACT: self.animationReact,
                             self.ACTION: self.animationAction,
                             self.RESTING: self.animationResting,
                             self.STANDUP: self.animationStandUp}

    # Physical Specifications
    
    self.fig_height     = 0.0
    self.tot_mass       = 0.0
    self.trunk_mass     = 0.0
    self.arm_mass       = 0.0
    self.leg_mass       = 0.0
    self.limb_density   = 900
    self.joint_density  = 10
    self.pelvis_density = 800
    self.torso_density  = 600
    self.head_density   = 360
    
    # Trunk Specifications
    
    self.pelvis_width     = 0.4
    self.pelvis_height    = 0.2
    self.pelvis_thickness = 0.2
    self.pelvis_xdim      = self.pelvis_width
    self.pelvis_ydim      = self.pelvis_height
    self.pelvis_zdim      = self.pelvis_thickness
    self.waist_radius     = self.pelvis_thickness/2.0
    self.waist_height     = 0.2
    self.torso_width      = 0.8
    self.torso_height     = 1.4
    self.torso_thickness  = 0.2
    self.torso_xdim       = self.torso_width
    self.torso_ydim       = self.torso_height
    self.torso_zdim       = self.torso_thickness
    self.neck_radius      = self.torso_thickness/2.0
    self.neck_height      = 0.2
    self.head_radius      = 0.3
    
    # Leg Specifications
    
    self.leg_length     = 0.0
    self.hip_radius     = 0.125
    self.thigh_width    = 2.0*self.hip_radius
    self.thigh_length   = 1.2
    self.thigh_radius   = self.thigh_width/2.0
    self.thigh_xdim     = self.thigh_width
    self.thigh_ydim     = self.thigh_length
    self.thigh_zdim     = self.thigh_width
    self.knee_radius    = self.thigh_radius
    self.shin_width     = 0.16
    self.shin_length    = 1.0
    self.shin_radius    = self.shin_width/2.0
    self.shin_xdim      = self.shin_width
    self.shin_ydim      = self.shin_length
    self.shin_zdim      = self.shin_width
    self.ankle_radius   = self.shin_radius*1.2
    self.foot_width     = 0.2
    self.foot_length    = 0.25
    self.foot_thickness = 0.1
    self.foot_xdim      = self.foot_width
    self.foot_ydim      = self.foot_thickness
    self.foot_zdim      = self.foot_length
    self.ball_radius    = self.foot_thickness*0.48
    self.toes_width     = 0.2
    self.toes_length    = 0.15
    self.toes_thickness = 0.1
    self.toes_xdim      = self.toes_width
    self.toes_ydim      = self.toes_thickness
    self.toes_zdim      = self.toes_length
    
    # Arm Specifications
    
    self.arm_length       = 0.0
    self.etow_length      = 0.0
    self.shoulder_radius  = 0.15
    self.upper_arm_width  = 0.16
    self.upper_arm_length = 0.6
    self.upper_arm_radius = self.upper_arm_width/2.0
    self.upper_arm_xdim   = self.upper_arm_width
    self.upper_arm_ydim   = self.upper_arm_length
    self.upper_arm_zdim   = self.upper_arm_width
    self.elbow_radius     = self.upper_arm_radius*sqrt(2.0)
    self.fore_arm_width   = 0.1
    self.fore_arm_length  = 0.8
    self.fore_arm_radius  = self.fore_arm_width/2.0
    self.fore_arm_xdim    = self.fore_arm_width
    self.fore_arm_ydim    = self.fore_arm_length
    self.fore_arm_zdim    = self.fore_arm_width
    self.wrist_radius     = self.fore_arm_radius*1.2
    self.hand_width       = 0.2
    self.hand_length      = 0.3
    self.hand_thickness   = 0.08
    self.hand_xdim        = self.hand_thickness
    self.hand_ydim        = self.hand_length
    self.hand_zdim        = self.hand_width
    
    # Figure Solids
    
    self.head        = None
    self.torso       = None
    self.waist       = None
    self.pelvis      = None
    self.r_hip       = None
    self.l_hip       = None
    self.r_thigh     = None
    self.l_thigh     = None
    self.r_knee      = None
    self.l_knee      = None
    self.r_shin      = None
    self.l_shin      = None
    self.r_ankle     = None
    self.l_ankle     = None
    self.r_foot      = None
    self.l_foot      = None
    self.r_ball      = None
    self.l_ball      = None
    self.r_toes      = None
    self.l_toes      = None
    self.r_shoulder  = None
    self.l_shoulder  = None
    self.r_upper_arm = None
    self.l_upper_arm = None
    self.r_elbow     = None
    self.l_elbow     = None
    self.r_fore_arm  = None
    self.l_fore_arm  = None
    self.r_wrist     = None
    self.l_wrist     = None
    self.r_hand      = None
    self.l_hand      = None
    
    # Figure Joints and Motor Specifications
    
    self.JointSpecs = \
    { 'ball' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : (-1.0, 0.0, 0.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'L-0' : {'mode' : 1, 'axis' : (-1.0, 0.0, 0.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)}
                 },
        'LoStop' : [ -30.0*RPD, 0.0, 0.0],
        'HiStop' : [   1.0*RPD, 0.0, 0.0],
        'Dratio' : [    0.1, 0.0, 0.0],
        'FMax'   : [  200.0, 0.0, 0.0],
        'AMFMax' : [ 2000.0, 0.0, 0.0]
      },
      'ankle' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : (-1.0, 0.0, 0.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'L-0' : {'mode' : 1, 'axis' : (-1.0, 0.0, 0.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 0.0,-1.0, 0.0)}
                 },
        'LoStop' : [-20.0*RPD, 0.0, -20.0*RPD],
        'HiStop' : [ 60.0*RPD, 0.0,  20.0*RPD],
        'Dratio' : [    0.1, 0.0,    0.1],
        'FMax'   : [  200.0, 0.0,  200.0],
        'AMFMax' : [ 2000.0, 0.0, 4000.0]
      },
      'knee' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : (-1.0, 0.0, 0.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'L-0' : {'mode' : 1, 'axis' : (-1.0, 0.0, 0.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)}
                 },
        'LoStop' : [   0.0*RPD, 0.0, 0.0],
        'HiStop' : [ 135.0*RPD, 0.0, 0.0],
        'Dratio' : [    0.2, 0.0, 0.0],
        'FMax'   : [ 1000.0, 0.0, 0.0],
        'AMFMax' : [10000.0, 0.0, 0.0]
      },
      'hip' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : ( 0.0, 0.0, 1.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 1.0, 0.0, 0.0)},
                   'L-0' : {'mode' : 1, 'axis' : ( 0.0, 0.0,-1.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 0.0,-1.0, 0.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 1.0, 0.0, 0.0)}
                 },
        'LoStop' : [-10.0*RPD, -30.0*RPD, -30.0*RPD],
        'HiStop' : [ 30.0*RPD,  30.0*RPD, 120.0*RPD],
        'Dratio' : [    0.2,     0.2,    0.2],
        'FMax'   : [  1000.0, 1000.0, 1000.0],
        'AMFMax' : [  8000.0, 8000.0,20000.0]
      },
      'wrist' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : ( 0.0, 0.0, 1.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 1.0, 0.0, 0.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'L-0' : {'mode' : 1, 'axis' : ( 0.0, 0.0,-1.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 1.0, 0.0, 0.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)}
                 },
        'LoStop' : [-90.0*RPD, 0.0, 0.0],
        'HiStop' : [ 60.0*RPD, 0.0, 0.0],
        'Dratio' : [   0.5, 0.0, 0.0],
        'FMax'   : [  75.0, 0.0, 0.0],
        'AMFMax' : [ 150.0, 0.0, 0.0]
      },
      'elbow' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : ( 1.0, 0.0, 0.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'L-0' : {'mode' : 1, 'axis' : ( 1.0, 0.0, 0.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 0.0,-1.0, 0.0)}
                 },
        'LoStop' : [  0.0*RPD, 0.0, -90.0*RPD],
        'HiStop' : [120.0*RPD, 0.0,  90.0*RPD],
        'Dratio' : [   0.1, 0.0,    0.1],
        'FMax'   : [1000.0, 0.0, 1000.0],
        'AMFMax' : [4000.0, 0.0, 4000.0]
      },
      'shoulder' : \
      { 'Axes' : { 'R-0' : {'mode' : 1, 'axis' : ( 0.0, 0.0, 1.0)},
                   'R-1' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                   'R-2' : {'mode' : 2, 'axis' : ( 1.0, 0.0, 0.0)},
                   'L-0' : {'mode' : 1, 'axis' : ( 0.0, 0.0,-1.0)},
                   'L-1' : {'mode' : 2, 'axis' : ( 0.0,-1.0, 0.0)},
                   'L-2' : {'mode' : 2, 'axis' : ( 1.0, 0.0, 0.0)}
                 },
        'LoStop' : [-20.0*RPD, -45.0*RPD, -60.0*RPD],
        'HiStop' : [120.0*RPD,  60.0*RPD, 120.0*RPD],
        'Dratio' : [    0.2,    0.2,    0.2],
        'FMax'   : [ 3000.0, 2000.0, 3000.0],
        'AMFMax' : [ 6000.0, 4000.0, 6000.0]
      },
      'waist' : \
      { 'Axes' : { 'C-0' : {'mode' : 1, 'axis' : ( 1.0, 0.0, 0.0)},
                   'C-1' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'C-2' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                 },
        'LoStop' : [-10.0*RPD, -30.0*RPD, -60.0*RPD],
        'HiStop' : [ 45.0*RPD,  30.0*RPD,  60.0*RPD],
        'Dratio' : [    0.1,    0.3,    0.1],
        'FMax'   : [  500.0,  500.0,  500.0],
        'AMFMax' : [ 8000.0, 8000.0, 8000.0]
      },
      'neck' : \
      { 'Axes' : { 'C-0' : {'mode' : 1, 'axis' : ( 1.0, 0.0, 0.0)},
                   'C-1' : {'mode' : 2, 'axis' : ( 0.0, 0.0, 1.0)},
                   'C-2' : {'mode' : 2, 'axis' : ( 0.0, 1.0, 0.0)},
                 },
        'LoStop' : [-45.0*RPD, -20.0*RPD, -60.0*RPD],
        'HiStop' : [ 45.0*RPD,  20.0*RPD,  60.0*RPD],
        'Dratio' : [    0.5,    0.5,    0.5],
        'FMax'   : [  500.0,  500.0,  500.0],
        'AMFMax' : [ 1000.0, 1000.0, 1000.0]
      }
    }
    
  def delete(self):
    
    self.origin = (0.0, 0.0, 0.0)
    
    self.frame.delete()

    self.body       = None
    self.target     = None

    self.head       = None
    self.neck       = None
    self.torso      = None
    self.waist      = None
    self.pelvis     = None
    self.r_hip      = None
    self.l_hip      = None
    self.r_thigh    = None
    self.l_thigh    = None
    self.r_knee     = None
    self.l_knee     = None
    self.r_shin     = None
    self.l_shin     = None
    self.r_ankle    = None
    self.l_ankle    = None
    self.r_foot     = None
    self.l_foot     = None
    self.r_ball     = None
    self.l_ball     = None
    self.r_toes     = None
    self.l_toes     = None
    self.r_shoulder = None
    self.l_shoulder = None
    self.r_elbow    = None
    self.l_elbow    = None
    self.r_wrist    = None
    self.l_wrist    = None
    self.r_hand     = None
    self.l_hand     = None

  def printPhysicalProperties(self):
    
    print("........ Figure Physical Properties ........")
    print(" ")
    print("Figure Height = %8.3f m" % (self.getFigureHeight()) )
    print("   Arm Length = %8.3f m" % (self.getExtendedArmLength()) )
    print("   Leg Length = %8.3f m" % (self.getExtendedLegLength()) )
    print(" ")
    print("Figure Mass = %8.3f kg" % (self.getTotMass()) )
    print(" Trunk Mass = %8.3f kg" % (self.getTrunkMass()) )
    print("   Arm Mass = %8.3f kg" % (self.getArmMass()) )
    print("   Leg Mass = %8.3f kg" % (self.getLegMass()) )
    print(" ")
    print("Ref. Pos.   = (%8.3f, %8.3f, %8.3f)" % \
          vecSub(self.refpos,self.origin) )
    print("CofM Pos.   = (%8.3f, %8.3f, %8.3f)" % \
          vecSub(self.calcCenterOfMass(),self.origin) )
    print("Pelvis Pos. = (%8.3f, %8.3f, %8.3f)" % \
          vecSub(self.pelvis.body.getPosition(),self.origin) )      
    print("Waist Pos.  = (%8.3f, %8.3f, %8.3f)" % \
          vecSub(self.waist.body.getPosition(),self.origin) )
    print(" ")
    self.printSolidProperties()
    print("............................................")

  def printSolidProperties(self):
      
    htot = self.getFigureHeight()
    mtot = self.getTotMass()
    
    print("                      %Total          %Total")
    print("Figure Solid  Height  Height   Mass    Mass ")
    print("------------  ------  ------  ------  ------")
    for s in self.frame.solids :
      h = solidHeight(s)
      m = s.mass
      print("%12s  %6.3f  %6.2f  %6.2f  %6.2f" % \
        (s.label, h, 100*(h/htot), m, 100*(m/mtot)) )
    print("------------  ------  ------  ------  ------")
     
  def create(self, px, py, pz):
    
    self.createFrame(px, py, pz)
    
  def createFrame(self, px, py, pz):
    
    (refpos) = self.frame.createFigureFrame(self, px, py, pz)
    
    self.refpos = refpos
    
    self.setArmMass()
    self.setLegMass()
    self.setTrunkMass()
    self.setTotMass()
    self.setElbowToWristLength()
    self.setExtendedArmLength()
    self.setExtendedLegLength()
    self.setFigureHeight()
    
    self.printPhysicalProperties()
        
    self.setAnimationState(self.INITIALIZE)
    
  def getRefPos(self):
    
    return self.refpos
    
  def resetConfig(self):
    
    self.origin = (0.0, 0.0, 0.0)
    self.refpos = (0.0, 0.0, 0.0)

    self.cgp_r_foot = []
    self.cop_r_foot = (0.0, 0.0, 0.0)
    self.cgp_l_foot = []
    self.cop_l_foot = (0.0, 0.0, 0.0)
    self.zmp        = (0.0, 0.0, 0.0)

    self.last_state = None
    self.body       = None
    self.target     = None

    self.frame.resetConfig()
    self.control.resetConfig()
    self.actions.resetConfig()
    
  def printConfig(self):
    
    self.frame.printConfig()
    self.control.printConfig()
    self.actions.printConfig()
    
  def setControlDebugOn(self)  : self.control.debugOn()
  def setControlDebugOff(self) : self.control.debugOff()
  def toggleControlDebug(self) : self.control.toggleDebug()
  def setActionsDebugOn(self)  : self.actions.debugOn()
  def setActionsDebugOff(self) : self.actions.debugOff()
  def toggleActionsDebug(self) : self.actions.toggleDebug()
    
  def setAnimationState(self, state):

    if not (state == self.state) :
      print("Setting animation state to %s" % ANIMATION_STATES[state])
    self.last_state = self.state
    self.state = state
    
  def getAnimationState(self):

    return self.state
    
  def resetAnimation(self):
    
    self.actions.resetConfig()
    self.actions.printConfig()
    self.setAnimationState(self.INITIALIZE)
    self.last_state = None
    self.body       = None
    self.target     = None

  def currStateJointManip(self):

    return self.state == self.JOINT_MANIP

  def prevStateJointManip(self):

    return self.last_state == self.JOINT_MANIP

  def doKeyPress(self, key):
  
    if self.frame.setConfig(key) :
      # Changes in frame configuration
      # necessitates animation reset
      self.resetAnimation()
      return True
      
    if self.frame.setState(key) : 
      return True
      
    if self.control.setTorqueControl(key) : 
      return True
      
    if self.actions.selectAction(key) :
      if self.actions.isStandingUp() :
        self.control.setTorqueModeOff()
        self.setAnimationState(self.STANDUP)
      if self.actions.inSuspendMode :
        self.setAnimationState(self.SUSPENDED) 
      if self.actions.isReaching() :
        self.setAnimationState(self.ACTION)
      if self.actions.isWalking() :
        self.setAnimationState(self.ACTION)
      return True
      
    return False

  def animationInitialize(self):
    # Initialize state -- enable zero error torque
    self.control.setTorqueModeOn()
    self.control.setZeroErrorOn()
    if self.actions.inSuspendMode() :
      self.setAnimationState(self.SUSPENDED)
    else :
      self.setAnimationState(self.STANDFIXED)
    # Invoke initial animation method
    self.invokeAnimationMethod(self.AnimationMethods[self.state])

  def animationStandStill(self):
    # Stand still state -- the figure is initially
    # standing upright with it's head at the reference
    # position.  It's posture is maintained by zero
    # error restoring torques applied to all joints,
    # but no attempt is made to keep the figure upright.
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationStandFixed(self):
    # Stand fixed state -- the figure is standing
    # with it's head initially at the reference position.
    # This upright posture is maintained by applying
    # torques to the waist, hip, knee and ankle joints
    # in order to keep the figure balanced with its
    # center of mass position located in the figure's
    # sagittal plane, and the center of gravity in the
    # ground plane between the ankle joints and midpoints
    # of the feet. The head remains level and pointed
    # forward, the torso is prevencted from swiveling,
    # but the arms and hands are free to move about.
    self.actions.standFixed(self.t, self.tstep)
    self.control.setZeroErrorOff()
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationSuspended(self):
    # Suspended state -- use force applied to
    # figure's head to keep the figure standing
    # with its head at the reference position
    self.actions.standSuspended()
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationJointManip(self):
    # Figure joint grabbed
    if self.actions.inSuspendMode():
      # apply external force to figure's head
      self.actions.standSuspended()
    # Apply joint motor torques and damping
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationSolidManip(self):
    # Figure body grabbed -- use force applied to
    # figure's head to keep the figure standing
    # with its head at the reference position
    if self.actions.inSuspendMode():
      # apply external force to figure's head
      self.actions.standSuspended()
    # Apply apply forces, torques and damping
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationReact(self):
    # Let figure react to target motion -- apply
    # forces, torques and damping
    self.actions.performAction(self.target, self.t, self.tstep)
    self.control.setZeroErrorOff()
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationAction(self):
    # Let figure perform selected action -- apply
    # forces, torques and damping
    self.actions.performAction(self.target, self.t, self.tstep)
    self.control.setZeroErrorOff()
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationResting(self):
    # Resting state -- the figure's posture depends
    # upon joint torque mode and zero error restoring
    # states. If torque mode is OFF, the figure is
    # limp, like a rag doll; if ON and zero error
    # restoring is ON, the figure will return to its
    # initial posture but not necessarily its initial
    # position
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def animationStandUp(self):
    # Standing up state -- move figure's head to
    # new reference position
    if self.actions.standUp():
      # Figure is now standing -- transition to
      # initialize state
      self.setAnimationState(self.INITIALIZE)
    self.control.applyMotorTorques(self.t, self.tstep)
    self.control.applyJointDamping(self.t, self.tstep)

  def invokeAnimationMethod(self, method):
    if method : method()

  def updateAnimation(self, body, target, t, tstep):

    self.body = body
    self.target = target
    self.t = t
    self.tstep = tstep
     
    if not self.frame.joints : 
      # Figure has collapsed
      return
    
    # Update figure animation state
    if body : 
      if body == target : 
        self.setAnimationState(self.REACT)
      else :
        if body.solid.joint : 
          self.setAnimationState(self.JOINT_MANIP)
          self.control.setManipJoint(body.solid.joint)
        else : 
          self.setAnimationState(self.SOLID_MANIP)
    else :
      if self.state == self.REACT :
        # Figure was reacting - transition to 
        # action, suspended or resting state
        if self.actions.isReaching() :
          self.setAnimationState(self.ACTION)
        elif self.actions.inSuspendMode() :
          self.setAnimationState(self.SUSPENDED)
        elif self.actions.isStanding() :
          self.setAnimationState(self.STANDFIXED)
        else :
          self.setAnimationState(self.RESTING)
      elif self.state == self.ACTION :
        if not ( self.actions.isWalking() or
                 self.actions.isReaching()    ) :
          # Figure was walking or reaching - 
          # transition to standup state
          self.control.setTorqueModeOff()
          self.setAnimationState(self.STANDUP)
      elif self.state == self.STANDSTILL :
        if self.actions.inSuspendMode() :
          # Figure was standing still - transition 
          # to suspended state
          self.setAnimationState(self.SUSPENDED)
      elif self.state == self.SUSPENDED :
        if not self.actions.inSuspendMode() :
          # Figure was suspended - transition
          # to standing still state
          self.control.setTorqueModeOn()
          self.control.setZeroErrorOn()
          self.setAnimationState(self.STANDSTILL)

    # Invoke animation method corresponding to current state
    self.invokeAnimationMethod(self.AnimationMethods[self.state])

  def setFigureHeight(self):
    
    htot = self.foot_thickness   + \
           self.ankle_radius*2.0 + \
           self.shin_length      + \
           self.knee_radius*2.0  + \
           self.thigh_length     + \
           self.hip_radius       + \
           self.pelvis_height    + \
           self.waist_height     + \
           self.torso_height     + \
           self.neck_height      + \
           self.head_radius*2.0
             
    self.fig_height = htot*self.sfac
    
  def getFigureHeight(self):
    
    return self.fig_height
    
  def setElbowToWristLength(self):
    
    dlen = self.elbow_radius     + \
           self.fore_arm_length  + \
           self.wrist_radius
           
    self.etow_length = dlen*self.sfac
    
  def getElbowToWristLength(self):
    
    return self.etow_length
        
  def setExtendedArmLength(self):
    
    darm = self.shoulder_radius  + \
           self.upper_arm_length + \
           self.elbow_radius*2.0 + \
           self.fore_arm_length  + \
           self.wrist_radius*2.0 + \
           self.hand_length      # to finger tips
           
    self.arm_length = darm*self.sfac
    
  def getExtendedArmLength(self):
    
    return self.arm_length

  def setExtendedLegLength(self):
    
    dleg = self.hip_radius       + \
           self.thigh_length     + \
           self.knee_radius*2.0  + \
           self.shin_length      + \
           self.ankle_radius*2.0 + \
           self.foot_thickness   # to heel
             
    self.leg_length = dleg*self.sfac
    
  def getExtendedLegLength(self):
    
    return self.leg_length
    
  def setArmMass(self):
    
    mass = self.r_shoulder.mass  + \
           self.r_upper_arm.mass + \
           self.r_elbow.mass     + \
           self.r_fore_arm.mass  + \
           self.r_wrist.mass     + \
           self.r_hand.mass
           
    self.arm_mass = mass
             
  def getArmMass(self):
    
    return self.arm_mass
    
  def setLegMass(self):
    
    mass = self.r_hip.mass   + \
           self.r_thigh.mass + \
           self.r_knee.mass  + \
           self.r_shin.mass  + \
           self.r_ankle.mass + \
           self.r_foot.mass  + \
           self.r_ball.mass  + \
           self.r_toes.mass
             
    self.leg_mass = mass
    
  def getLegMass(self):
    
    return self.leg_mass
  
  def setTrunkMass(self):
    
    mass = self.pelvis.mass + \
           self.waist.mass  + \
           self.torso.mass  + \
           self.neck.mass  
             
    self.trunk_mass = mass
    
  def getTrunkMass(self):
    
    return self.trunk_mass
  
  def setTotMass(self):
    
    totmass = 0.0
    for s in self.frame.solids :
      totmass += s.mass
      
    self.tot_mass = totmass
    
  def getTotMass(self):
    
    return self.tot_mass
  
  def calcCenterOfMass(self):
    
    x = 0.0
    y = 0.0
    z = 0.0
    
    for s in self.frame.solids :
      bpos = s.body.getPosition()
      mass = s.body.mass
      x += mass*bpos[0]
      y += mass*bpos[1]
      z += mass*bpos[2]
    
    totmass = self.getTotMass()
    
    x = x/totmass
    y = y/totmass
    z = z/totmass
    
    return (x, y, z)
    
  def calcCenterOfPressure(self, foot):
    
    x = 0.0
    y = 0.0
    z = 0.0
    
    if foot == 'R' : cgp = self.cgp_r_foot
    else           : cgp = self.cgp_l_foot
    
    n = 0
    for (pos,nrm) in cgp :
      x += pos[0]
      z += pos[2]
      n += 1

    if n > 0 :
      x = x/n
      z = z/n
    
    return (x, y, z)
    
  def setCOP(self, foot):
      
    cop = self.calcCenterOfPressure(foot)
    
    if foot == 'R' : self.cop_r_foot = cop
    else           : self.cop_l_foot = cop
      
  def getCOP(self, foot):
      
    if foot == 'R' : return self.cop_r_foot
    else           : return self.cop_l_foot
    
  def appendCGP(self, foot, cgp):
    
    if foot == 'R' : self.cgp_r_foot.append(cgp)
    else           : self.cgp_l_foot.append(cgp)
  
  def createCGP(self, foot):
      
    if foot == 'R' : self.cgp_r_foot = []
    else           : self.cgp_l_foot = []
    
  def calcZMP(self):
      
    #print("=== calcZMP:")
    
    sumMx = 0.0
    sumMz = 0.0
    sumFy = 0.0
    
    for j in self.frame.joints:
      name = j.label
      b1   = j.getBody(0)
      b2   = j.getBody(1)
      if not isinstance(j,ode.FixedJoint) :
        if isinstance(j,ode.FixedJoint) :
          j1Pos = b2.getPosition()
          j2Pos = b2.getPosition()
        else :
          j1Pos = j.getAnchor()
          j2Pos = j.getAnchor2()
        b1Pos = b1.getPosition()
        R1    = vecSub(j1Pos,b1Pos)
        r1    = vecMag(R1)
        b2Pos = b2.getPosition()
        R2    = vecSub(j2Pos,b2Pos)
        r2    = vecMag(R2)
        fb    = j.getFeedback()
        if fb :
          F1 = fb[0]
          T1 = fb[1]
          F2 = fb[2]
          T2 = fb[3]
          if (True not in [isnan(F1[i]) for i in range(len(F1))]) and \
             (True not in [isnan(T1[i]) for i in range(len(T1))]) :
            f1sq = vecMagSq(F1)
            t1sq = vecMagSq(T1)
            if ((not isinf(f1sq)) and (f1sq >= 0.0)) and \
               ((not isinf(t1sq)) and (t1sq >= 0.0)) :
              """
              f1 = vecMag(F1)
              t1 = vecMag(T1)
              f2 = vecMag(F2)
              t2 = vecMag(T2)
              m1 = b1.solid.mass
              m2 = b2.solid.mass
              print("%s : " % name)
              print("%s : %8.2f %8.3f %9.3f %9.3f" % (b1.solid.label,m1,r1,f1,t1) )
              print("  F1 = %9.3f %9.3f %9.3f  T1 = %9.3f %9.3f %9.3f" % \
                    (F1[0],F1[1],F1[2],T1[0],T1[1],T1[2]) )
              print("  R1xF1 = %9.3f %9.3f %9.3f " % vecCrossP(R1,F1) )
              print("%s : %8.2f %8.3f %9.3f %9.3f" % (b2.solid.label,m2,r2,f2,t2) )
              print("  F2 = %9.3f %9.3f %9.3f  T2 = %9.3f %9.3f %9.3f" % \
                    (F2[0],F2[1],F2[2],T2[0],T2[1],T2[2]) )
              print("  R2xF2 = %9.3f %9.3f %9.3f " % vecCrossP(R2,F2) )
              """
              sumMx = sumMx + R1[1]*F1[2] - R1[2]*F1[1] + T1[0]
              sumMz = sumMz + R1[0]*F1[1] - R1[1]*F1[0] + T1[2]
              sumFy = sumFy + F1[1]
              """
              sumMx = sumMx + R2[1]*F2[2] - R2[2]*F2[1] + T2[0]
              sumMz = sumMz + R2[0]*F2[1] - R2[1]*F2[0] + T2[2]
              sumFy = sumFy + F2[1]
              """
          
    if ( sumFy == 0.0 ) :
      (x, y, z) = self.calcCenterOfMass()
    else :
      x = sumMz/sumFy
      y = 0.0
      z = sumMx/sumFy
     
    #print("ZMP = %f %f %f" % (x,y,z) )
    return (x, 0.0, z)
    
  def setZMP(self,zmp):
    
    self.zmp = zmp
    
  def getZMP(self):
    
    return self.zmp
