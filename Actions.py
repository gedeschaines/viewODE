# Actions.py
#
# viewODE humanoid robotic figure actions definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Disclaimer
#
#   See the file DISCLAIMER-GaryDeschaines

from sys import exit
from math import asin
#
# Import ODE module for joint and motor modeling.

try:
  import ode
except:
  print("Error: This module requires PyODE !!")
  exit(0)

#
# Import viewODE modules for humanoid figure joints, motors and vector math.

try:
  from Joints import *
except:
  print("Error: Joints not installed properly !!")
try:
  from Motors import *
except:
  print("Error: Motors not installed properly !!")
try:
  from vecMath import *
except:
  print("Error: vecMath not installed properly !!")
  
class Actions :
  
  def __init__(self, figure):
    """ Constructor.
        figure -- viewODE figure
                    
        Initialize the viewODE figure actions.
    """
    self.figure      = figure
    self.frame       = figure.frame
    self.debug       = False
    self.suspendmode = False
    self.stridetest  = False
    self.standing    = True
    self.standingup  = False
    self.walking     = False
    self.kicking     = False
    self.reaching    = False
    self.angerr_tol  = 0.5*RPD
    self.angvel_max  = TWOPI/2.0
    self.t0          = 0.0
    self.last_cmpos  = []
    self.last_cmvel  = []
    
  def debugOn(self)     : self.debug = True
  def debugOff(self)    : self.debug = False
  def toggleDebug(self) : self.debug = not self.debug
    
  def setSuspendModeOn(self):
    if not self.suspendmode :
      self.suspendmode = True
      self.standingup  = False
      self.printConfig()
    
  def setSuspendModeOff(self):
    if self.suspendmode :
      self.suspendmode = False
      self.printConfig()
    
  def toggleSuspendMode(self):
    if not self.suspendmode : self.setSuspendModeOn()
    else                    : self.setSuspendModeOff()
      
  def inSuspendMode(self):
    return self.suspendmode
  
  def setStrideTestOn(self):
    if not self.stridetest :
      self.stridetest = True
      print("Stride test mode active")
    
  def setStrideTestOff(self):
    if self.stridetest :
      self.stridetest = False
      self.figure.floor.enable()
      print("Stride test mode inactive")
            
  def toggleStrideTest(self):
    if self.stridetest : self.setStrideTestOff()
    else               : self.setStrideTestOn()
    
  def inStrideTest(self):
    return self.stridetest
      
  def isStanding(self):
    return self.standing
    
  def isReaching(self):
    return self.reaching
    
  def isKicking(self):
    return self.kicking
    
  def isWalking(self):
    return self.walking
  
  def isStandingUp(self):
    return self.standingup
  
  def setKickOn(self):
    if not self.kicking :
      self.kicking    = True     
      self.reaching   = False
      self.walking    = False
      self.standingup = False
      self.printConfig()
      
  def setKickOff(self):
    if self.kicking : 
      self.kicking = False
      self.printConfig()
  
  def toggleKick(self):
    if self.kicking : self.setKickOff()
    else            : self.setKickOn()
      
  def setReachOn(self):
    if not self.reaching : 
      self.reaching   = True
      self.kicking    = False
      self.walking    = False
      self.standingup = False
      self.printConfig()
      
  def setReachOff(self):
    if self.reaching : 
      self.reaching = False
      self.printConfig()
  
  def toggleReach(self):
    if self.reaching : self.setReachOff()
    else             : self.setReachOn()
      
  def setWalkOn(self):
    if self.standing and not self.walking : 
      self.walking     = True
      self.suspendmode = False
      self.reaching    = False
      self.kicking     = False
      self.standing    = False
      self.standingup  = False
      self.last_cmpos  = []
      self.last_cmvel  = []
      self.t0          = 0.0
      self.printConfig()
      
  def setWalkOff(self):
    if self.walking : 
      self.walking    = False
      self.standingup = True
      self.setStrideTestOff()
      self.printConfig()
      
  def toggleWalk(self):
    if self.walking : self.setWalkOff()
    else            : self.setWalkOn()
    
  def resetConfig(self):
    self.suspendmode = False
    self.stridetest  = False
    self.standing    = True
    self.standingup  = False
    self.walking     = False
    self.kicking     = False
    self.reaching    = False
    self.last_cmpos  = []
    self.last_cmvel  = []
    self.figure.floor.enable()
    
  def printConfig(self):
    
    print("----- Figure Actions -----")
    if self.suspendmode : print("Suspend mode active")
    else                : print("Suspend mode inactive")
    if self.stridetest  : print("Stride test mode active")
    else                : print("Stride test mode inactive")    
    if self.standing    : print("Standing active")
    else                : print("Standing inactive")
    if self.standingup  : print("Standing up action active")
    else                : print("Standing up action inactive")
    if self.walking     : print("Walking action active")
    else                : print("Walking action inactive")
    if self.kicking     : print("Kicking action active")
    else                : print("Kicking action inactive")
    if self.reaching    : print("Reaching action active")
    else                : print("Reaching action inactive")

  def selectAction(self, key):
    
    got = False
  
    if key == '\x20' :
      # Toggle suspend mode
      got = True
      self.toggleSuspendMode()
    elif key == 'y' or key == 'Y' :
      # Toggle stride test mode
      got = True
      self.toggleStrideTest()
    elif key == 'k' or key == 'K' :
      # Toggle kick action
      got = True
      self.toggleKick()
    elif key == 'r' or key == 'R' :
      # Toggle reach action
      got = True
      self.toggleReach()
    elif key == 'u' or key == 'U' :
      # Have the target stand up
      got = True
      if not (self.standingup and self.standing) :
        self.standingup = True
        self.walking    = False
        self.printConfig()
    elif key == 'w' or key == 'W' :
      # Toggle walking/standing action
      got = True
      self.toggleWalk()
      
    return got
    
  def performAction(self, target, t, tstep):
    
    if self.suspendmode :
      self.standSuspended()
      
    if self.standingup :
      self.standUp()
      
    if self.walking :
      self.Walk(t, tstep)
      
    if self.kicking :
      see = self.turnHeadTowardTarget(target, t, tstep)
      if see :
        facing = self.turnTorsoTowardTarget(target, t, tstep)
        if facing :
          self.kickFootTowardTarget(self.figure.r_foot,
                                    target, t, tstep)
      
    if self.reaching :
      self.standFixed(t, tstep)
      see = self.turnHeadTowardTarget(target, t, tstep)
      if see :
        facing = self.turnTorsoTowardTarget(target, t, tstep)
        if facing :
          self.reachHandTowardTarget(self.figure.r_shoulder,
                                     target, t, tstep)
          self.reachHandTowardTarget(self.figure.l_shoulder,
                                     target, t, tstep)                    
          # self.reachHandsTowardTarget(target, t, tstep)
                                     
  def calcAngularVelCMwrtB(self, CMpos, CMvel, B):
    """ Returns angular velocity vector normal to
        the relative position vector from B to CM
    """
    Bpos = B.getPosition()
    Bvel = B.getLinearVel()
    Banv = B.getAngularVel()
        
    Rpos = vecSub(CMpos,Bpos)
    Rvel = vecSub(vecSub(CMvel,Bvel),vecCrossP(Banv,Rpos))
    Wvec = vecDivS(vecCrossP(Rpos, Rvel),vecMagSq(Rpos))
    
    return Wvec
    
  def calcAngularVelAwrtB(self, A, B):
    """ Returns angular velocity vector normal to
        the relative position vector from B to A
    """
    Apos = A.getPosition()
    Avel = A.getLinearVel()
    Bpos = B.getPosition()
    Bvel = B.getLinearVel()
    Banv = B.getAngularVel()
        
    Rpos = vecSub(Apos,Bpos)
    Rvel = vecSub(vecSub(Avel,Bvel),vecCrossP(Banv,Rpos))
    Wvec = vecDivS(vecCrossP(Rpos, Rvel),vecMagSq(Rpos))
    
    return Wvec
    
  def calcLosRateAwrtB(self, A, B):
    """ Returns angular velocity vector normal to
        the line-of-sight vector from B to A
    """
    Apos = A.getPosition()
    Avel = A.getLinearVel()
    Bpos = B.getPosition()
    Bvel = B.getLinearVel()
        
    Vrel   = vecSub(Avel,Bvel)
    Rlos   = vecSub(Apos,Bpos)
    Ulos   = unitVec(Rlos)
    Vclose = projectVecAonUB(Vrel,Ulos)
    
    Wlos = vecDivS(vecCrossP(Ulos,vecSub(Vrel,Vclose)),vecMag(Rlos))
    
    return Wlos
    
  def standSuspended(self):
    """ Suspend figure from its reference position while in a 
        standing posture. 
    """ 
    fig  = self.figure
    hpos = fig.head.body.getPosition()
    uvec = unitVec(vecSub(fig.refpos, hpos))
    dvec = (uvec[0], uvec[1]+1.0, uvec[2])
    fmag = fig.getTotMass()*vecMag(fig.world.getGravity())
    fvec = vecMulS(dvec, fmag)
    fig.head.body.addForce(fvec)
    
  def standFixed(self,t,tstep):
    """ Maintain figure's balance while in a standing posture at 
        a fixed location.
        References:
        [1] Mario  Ricardo  Arbul´u  Saavedra  “Stable  locomotion  of  humanoid
            robots based on mass concentrated model” Ph. D thesis, October 2008
        [2] Dr. D. Kostic “Zero-moment point method for stable biped walking”
            Eindhoven, University of Technology, DCT no.: 2009.072, July 2009
    """
    fig = self.figure

    # Calculate gravity angle about world x-axis; same as forward slope angle
    gvec = fig.world.getGravity()
    angx = atan2(-gvec[2]/9.81, -gvec[1]/9.81)*DPR
    
    # Calculate figure's center of mass velocity and acceleration.
    cmpos = fig.calcCenterOfMass()
    if self.last_cmpos :
      cmvel = vecMulS(vecSub(cmpos,self.last_cmpos), 1.0/tstep)
      cmacc = vecMulS(vecSub(cmvel,self.last_cmvel), 1.0/tstep)
    else : 
      cmvel = (0.0, 0.0, 0.0)
      cmacc = (0.0, 0.0, 0.0)
      self.t0 = t  
    self.last_cmpos = cmpos
    self.last_cmvel = cmvel

    # Calculate center of gravity position wrt figure's origin.
    cgpos = vecSub(cmpos, fig.origin)
    
    # Calculate the zero moment point on the ground plane as
    # measured from the figure's origin (eqs. 3.20 & 3.21
    # from ref [2]).
    zmpx = cgpos[0] - (cgpos[1]/9.81)*cmacc[0]
    zmpy = 0.0
    zmpz = cgpos[2] - (cgpos[1]/9.81)*cmacc[2]
    fig.setZMP(vecAdd((zmpx, zmpy, zmpz),fig.origin))
    
    # Estimate where center of mass will be next half tstep.
    htstep = tstep/2.0
    cmvele = cmvel + vecMulS(cmacc,htstep)
    cmpose = cmpos + vecMulS(cmvel,htstep) + vecMulS(cmacc,0.5*htstep*htstep)
    
    # Calculate the forward distance and height of the 
    # center of mass with respect to the right and left 
    # ankles (assumes feet are flat on ground with toes
    # pointed along +Z world axis).
    cmprel = vecSub(cmpose,fig.r_ankle.body.getPosition())
    cmhgtr = cmprel[1]
    cmdstr = cmprel[2]
    cmprel = vecSub(cmpose,fig.l_ankle.body.getPosition())
    cmhgtl = cmprel[1]
    cmdstl = cmprel[2]
    cmdst  = 0.5*(cmdstr + cmdstl)
    
    # Calculate the average forward/backward and side-to-side 
    # angular velocities of the center of mass about the ankles
    # (assumes feet are flat on ground with toes pointed along
    # +Z world axis).
    cmvelZ = projectVecAonUB(cmvele,(0.0,0.0,1.0))
    angvXr = cmvelZ[2]/cmhgtr
    angvXl = cmvelZ[2]/cmhgtl
    angvX  = 0.5*(angvXr + angvXl)
    cmvelX = projectVecAonUB(cmvele,(1.0,0.0,0.0))
    angvZr = cmvelX[0]/cmhgtr
    angvZl = cmvelX[0]/cmhgtl
    angvZ  = 0.5*(angvZr + angvZl)
        
    # Get pelvis angular velocity about the world +X axis.
    pelvis = fig.pelvis.body
    p_pos  = vecSub(pelvis.getPosition(),fig.origin)
    p_vel  = pelvis.getLinearVel()
    pangv  = pelvis.getAngularVel()
    pangvX = vecDotP(pangv,(1.0, 0.0, 0.0))
       
    if self.debug :
      print("StandFixed:  t=%8.4f" % (t))
      print("  cmpos :  (%8.3f | %8.3f | %8.3f)" % cmpos)
      print("  cmvel :  (%8.3f | %8.3f | %8.3f)" % cmvel)
      print("  cmacc :  (%8.3f | %8.3f | %8.3f)" % cmacc)
      print("  cgpos :  (%8.3f | %8.3f | %8.3f)" % cgpos)
      print("  p_pos :  (%8.3f | %8.3f | %8.3f)" % p_pos)
      print("  p_vel :  (%8.3f | %8.3f | %8.3f)" % p_vel)
      print("  zmpnt :  %8.3f, %8.3f" % (zmpx, zmpz))
      print("  cmdst :  %8.3f, %8.3f" % (cmdstr, cmdstl))
      print("  cmhgt :  %8.3f, %8.3f" % (cmhgtr, cmhgtl))
      print("  angvX :  %8.3f" % (angvX))
      print("  angvZ :  %8.3f" % (angvZ))
      print("  pangvX:  %8.3f" % (pangvX))
      
    TOL  = self.angerr_tol
    RATE = self.angvel_max
    if fig.control.rotdamping : KFAC = 1.8
    else                      : KFAC = 1.2

    # Rotate arms to counter gravity angle.
    rs_motor = fig.r_shoulder.motor
    ls_motor = fig.l_shoulder.motor
    rotateAxisToAngle(rs_motor, 2, angx*RPD, TOL, RATE)
    rotateAxisToAngle(ls_motor, 2, angx*RPD, TOL, RATE)

    # Rotate head to keep eyes level and straight ahead. 
    n_joint = fig.neck.joint
    n_motor = fig.neck.motor
    # Rotate head to keep eyes traight ahead. 
    rotateAxisToZero(n_motor, 2, TOL, RATE)
    if isinstance(n_joint, ode.BallJoint) :
      # Keep head from wobbling side to side.
      #rotateAxisToZero(n_motor, 1, TOL, RATE)
      rotateAxisAtRate(n_motor, 1, -KFAC*angvZ)
      
    # Set the waist, hip, knee and ankle joint motor rotations
    # to keep the figure balanced in a standing posture.
    
    w_joint  = fig.waist.joint
    w_motor  = fig.waist.motor
    rh_motor = fig.r_hip.motor
    rk_motor = fig.r_knee.motor
    ra_motor = fig.r_ankle.motor
    rb_motor = fig.r_ball.motor
    lh_motor = fig.l_hip.motor
    lk_motor = fig.l_knee.motor
    la_motor = fig.l_ankle.motor
    lb_motor = fig.l_ball.motor
    
    # Keep Waist from swiveling.
    rotateAxisToZero(w_motor, 2, TOL, RATE)
    # Keep waist from bending side to side.
    if isinstance(w_joint, ode.BallJoint) :
      rotateAxisToZero(w_motor, 1, TOL, RATE)
      
    if cmdst > fig.foot_zdim :  # COM anterior to foot ball.
      # Rotate head to keep eyes level. 
      rotateAxisAtRate(n_motor, 0, -KFAC*angvX)
      # Apply rotation to waist to counter cm angular velocity.
      rotateAxisAtRate(w_motor, 0, -KFAC*0.5*(angvX+pangvX))
      # Apply rotation to hips to counter pelvis rotation.
      rotateAxisAtRate(rh_motor, 2, -KFAC*pangvX)
      rotateAxisAtRate(lh_motor, 2, -KFAC*pangvX)
      # Rotate ankles to 1 degree angle.
      rotateAxisToAngle(ra_motor, 0, (1.0-angx)*RPD, TOL, RATE)
      rotateAxisToAngle(la_motor, 0, (1.0-angx)*RPD, TOL, RATE)
    elif cmdst < 0.0 :  # COM posterior to ankle joint
      # Rotate head to keep eyes level. 
      rotateAxisAtRate(n_motor, 0, -KFAC*angvX)
      # Apply rotation to waist to counter cm angular velocity.
      rotateAxisAtRate(w_motor, 0, -KFAC*0.5*(angvX+pangvX))
      # Apply rotation to hips to counter pelvis rotation.
      rotateAxisAtRate(rh_motor, 2, -KFAC*pangvX)
      rotateAxisAtRate(lh_motor, 2, -KFAC*pangvX)
      # Rotate ankles to -3 degree angle.
      rotateAxisToAngle(ra_motor, 0, (-3.0-angx)*RPD, TOL, RATE)
      rotateAxisToAngle(la_motor, 0, (-3.0-angx)*RPD, TOL, RATE)
    else :  # COM posterior to foot midpoint and anterior to ankle joint
      # Rotate head to keep eyes level. 
      rotateAxisToAngle(n_motor, 0, 0.0*RPD, TOL, RATE)
      # Keep waist at -3 degree angle.
      rotateAxisToAngle(w_motor, 0, (-3.0+angx)*RPD, TOL, RATE)
      # Keep hips at -3 degree angle.
      rotateAxisToAngle(rh_motor, 2, -3.0*RPD, TOL, RATE)
      rotateAxisToAngle(lh_motor, 2, -3.0*RPD, TOL, RATE)
      # Apply rotation to ankles to counter forward/backward sway.
      rotateAxisToAngle(ra_motor, 0, (-3.0-angx)*RPD, TOL, RATE)
      rotateAxisToAngle(la_motor, 0, (-3.0-angx)*RPD, TOL, RATE)
      
    # Apply rotation to hips to counter side-to-side sway.
    rotateAxisAtRate(rh_motor, 0, -KFAC*angvZ)
    rotateAxisAtRate(lh_motor, 0,  KFAC*angvZ)

    # Keep knees locked to initial position (zero angle).
    rotateAxisToZero(rk_motor, 0, TOL, RATE)
    rotateAxisToZero(lk_motor, 0, TOL, RATE)
    
    # Keep toes locked to initial position (zero angle).
    rotateAxisToZero(rb_motor, 0, TOL, RATE)
    rotateAxisToZero(lb_motor, 0, TOL, RATE)
       
    # Splay feet outward to minimize side-to-side sway. 
    #rotateAxisToAngle(ra_motor, 2, 10*RPD, TOL, RATE)
    #rotateAxisToAngle(la_motor, 2, 10*RPD, TOL, RATE)

  def standUp(self):
    """ Return figure to a standing posture, suspended from
        its reference position. 
    """ 
    if not self.standingup : return
    
    fig  = self.figure
    hpos = fig.head.body.getPosition()
    ppos = fig.pelvis.body.getPosition()
    dvec = (       ppos[0]-hpos[0], \
             fig.refpos[1]-hpos[1], \
                   ppos[2]-hpos[2]  )
    dmag = vecMag(dvec)
    uvec = unitVec(dvec)
    dvec = (uvec[0], uvec[1]+1.0, uvec[2])

    fmag = fig.getTotMass()*vecMag(fig.world.getGravity())
    fvec = vecMulS(dvec, fmag)
    fig.head.body.addRelForce(fvec)
    
    if dmag < 0.01 and hpos[1] >= fig.refpos[1] :
      self.standing   = True
      self.standingup = False
      fig.refpos = (hpos[0], fig.refpos[1], hpos[2])
    else :
      self.standing = False

    return self.standing

  def heelTouchingGround(self, foot):
    
    (sx, sy, sz) = foot.boxsize
    rpos = (0.0, -sy/2.0, -sz/2.0) # Heel
    spos = foot.body.getRelPointPos( rpos )
    if spos[1] <= self.figure.foot_thickness/2.0 : return True
    else                                         : return False
          
  def toesTouchingGround(self, foot):
    
    (sx, sy, sz) = foot.boxsize
    rpos = (0.0, -sy/2.0, +sz/2.0) # Toes
    spos = foot.body.getRelPointPos( rpos )
    if spos[1] <= self.figure.toes_thickness/2.0 : return True
    else                                         : return False
      
  def pushFootAgainstGround(self, foot, ankle, ball, limit, tstep) :
    
    TOL  = self.angerr_tol
    RATE = self.angvel_max
    
    a_motor = ankle.motor
    b_motor = ball.motor
    
    if self.toesTouchingGround(foot) :
      # Foot touching ground -- rotate ankle towards limit, 
      # and flex toes upward
      rotateAxisToAngle(a_motor, 0, limit, TOL, RATE)
      rotateAxisToStop(b_motor, 0, 'LoStop', TOL, RATE)
      # Stiffen ankle
      rotateAxisToZero(a_motor, 2, TOL, RATE)
      return True
    else :
      # Foot not touching ground -- relax foot
      removeTorqueFromAxis(a_motor, 0)
      removeTorqueFromAxis(a_motor, 2)
      removeTorqueFromAxis(b_motor, 0)
      return False
      
  def synchKneeFootwithHip(self, hip, angv, anglim, tstep):
    
    TOL  = self.angerr_tol
    RATE = self.angvel_max
    
    if hip.joint.side == 'R' :
      knee  = self.figure.r_knee
      ankle = self.figure.r_ankle
      ball  = self.figure.r_ball
      foot  = self.figure.r_foot
    else :
      knee  = self.figure.l_knee
      ankle = self.figure.l_ankle
      ball  = self.figure.l_ball
      foot  = self.figure.l_foot
      
    k_motor = knee.motor
    a_motor = ankle.motor
    b_motor = ball.motor
 
    if angv < 0.0 : 
      # Leg moving backward
      ang = hip.motor.getAngle(2)
      if ang < 0.0 :
        # Leg behind hip
        if self.pushFootAgainstGround(foot, ankle, ball, pi/3.0, tstep) :
          # Stiffen knee
          rotateAxisToZero(k_motor, 0, TOL, RATE)
        else :
          rotateAxisToAngle(k_motor, 0, 2*anglim, TOL, RATE)
          rotateAxisToAngle(a_motor, 0, anglim, TOL, RATE)
          rotateAxisToZero(b_motor, 0, TOL, RATE)
      else :
        # Leg in front of hip
        rotateAxisToZero(k_motor, 0, TOL, RATE)
        rotateAxisToZero(a_motor, 0, TOL, RATE)
        rotateAxisToZero(a_motor, 2, TOL, RATE)
        rotateAxisToZero(b_motor, 0, TOL, RATE)
    else :
      # Leg moving forward
      ang = hip.motor.getAngle(2)
      if ang < 0.0 :
        # Leg behind hip
        rotateAxisToAngle(k_motor, 0, 2*anglim, TOL, RATE)
        rotateAxisToStop(a_motor, 0, 'LoStop', TOL, RATE)
        rotateAxisToStop(b_motor, 0, 'LoStop', TOL, RATE)
      else :
        # Leg in front of hip
        rotateAxisToAngle(k_motor, 0, anglim, TOL, RATE)
        rotateAxisToStop(a_motor, 0, 'LoStop', TOL, RATE)
        rotateAxisToZero(b_motor, 0, TOL, RATE)
        # Rotate foot slighty outward
        ang = a_motor.joint.specs['HiStop'][2]/2.0
        rotateAxisToAngle(a_motor, 2, ang, TOL, RATE)

  def synchArmMotionwithHip(self, shoulder, angv, anglim, tstep):
    
    TOL  = self.angerr_tol
    RATE = self.angvel_max
    
    s_joint = shoulder.joint
    s_motor = shoulder.motor
    
    # Swing arm for balance
    rotateAxisAtRate(s_motor, 2, angv)
    
    # Keep arm from flapping up and down
    rotateAxisToZero(s_motor, 0, TOL, RATE)
    
    if isinstance(s_joint, ode.BallJoint) :
      # Keep arm from twisting
      rotateAxisToZero(s_motor, 1, TOL, RATE)
        
  def rotateHip(self, hip, angv, anglim, tstep):
    
    TOL  = self.angerr_tol
    RATE = self.angvel_max
    
    h_joint = hip.joint
    h_motor = hip.motor
    pelvis  = self.figure.pelvis.body
    
    # Set hip motor angular velocity
    hjbody2 = h_joint.getBody(1)
    hipwvec = hjbody2.vectorFromWorld(pelvis.getAngularVel())
    maxis   = hjbody2.vectorFromWorld(h_motor.getAxis(2))
    hipwmag = vecDotP(hipwvec, maxis)
    rotateAxisAtRate(h_motor, 2, angv-hipwmag)
    
    # Prevent leg camber
    rotateAxisToZero(h_motor, 0, TOL, RATE)
        
    if isinstance(h_joint, ode.BallJoint) :
      # Keep leg from twisting
      rotateAxisToZero(h_motor, 1, TOL, RATE)

  def Walk(self, t, tstep):
    """ Walk at a fixed pace.
        References:
        [1] Mario  Ricardo  Arbul´u  Saavedra  “Stable  locomotion  of  humanoid
            robots based on mass concentrated model” Ph. D thesis, October 2008
        [2] Dr. D. Kostic “Zero-moment point method for stable biped walking”
            Eindhoven, University of Technology, DCT no.: 2009.072, July 2009
    """
    if not self.walking : return
    if not self.frame   : return
      
    TOL  = self.angerr_tol
    RATE = self.angvel_max
    
    if self.inStrideTest() :
      # Suspend body by head (for stride testing)
      self.standSuspended()
      # Disable floor to prevent foot collisions
      self.figure.floor.disable()

    # Calculate balancing force to keep figure upright.

    fig     = self.figure
    totmass = fig.getTotMass()
    cmpos   = fig.calcCenterOfMass()
    pelvis  = fig.pelvis.body

    # Calculate the zero moment point on the ground plane as
    # measured from the world space origin (eqs. 3.17 & 3.18
    # from ref [2]).
    zmp = fig.calcZMP()
    fig.setZMP(zmp)

    # Calculate center of gravity position wrt figure's torso position.
    torso = fig.torso.body
    tpos  = torso.getPosition()
    cgpos = vecSub(cmpos,tpos)

    # Calculate figure's center of mass velocity and acceleration.
    if self.last_cmpos :
      cmvel = vecMulS(vecSub(cmpos,self.last_cmpos), 1.0/tstep)
      cmacc = vecMulS(vecSub(cmvel,self.last_cmvel), 1.0/tstep)
    else : 
      cmvel   = (0.0, 0.0, 0.0)
      cmacc   = (0.0, 0.0, 0.0)
      self.t0 = t
    self.last_cmpos = cmpos
    self.last_cmvel = cmvel

    # Calculate the zero moment point on the ground plane as
    # measured from the world space origin (eqs. 3.20 & 3.21
    # from ref [2]).
    #zmpx = cmpos[0] - (cmpos[1]/9.81)*cmacc[0]
    #zmpy = 0.0
    #zmpz = cmpos[2] - (cmpos[1]/9.81)*cmacc[2]
    #zmp = (zmpx, zmpy, zmpz)
    fig.setZMP(zmp)

    # Calculate hip and waist angle limits and angular velocity
    # from desired walking speed and extended leg length
    speed      = 2.0
    dleg       = fig.getExtendedLegLength()
    anglim     = 0.5*(speed/dleg)
    anglimx2pi = anglim*TWOPI

    tdel   = t - self.t0
    rangv2 = sin(TWOPI*tdel+HALFPI)*anglimx2pi  # right hip ang vel
    langv2 = sin(TWOPI*tdel-HALFPI)*anglimx2pi  # left hip ang vel
    wangv  = langv2                             # waist ang vel
    
    # Rotate hips
    r_hip = fig.r_hip
    l_hip = fig.l_hip
    self.rotateHip(r_hip, rangv2, anglim, tstep)
    self.rotateHip(l_hip, langv2, anglim, tstep)
        
    # Apply artificial balancing torque and forward velocity --
    # allow torso to pivot around +z axis but try to keep torso
    # upright and facing forward.  Also, try to keep torso from
    # overtaking center of mass
    Wvec  = self.calcAngularVelCMwrtB(cmpos, cmvel, torso)
    Wmagx = vecDotP(Wvec, (1.0,0.0,0.0))
    Wmagy = vecDotP(Wvec, (0.0,1.0,0.0))
    Wvec  = (Wmagx, Wmagy, 0.0)
    wvel  = (0.0, 0.0, speed)
    # tvel = vecSub(wvel, torso.getLinearVel())
    # Vvec = vecSub(cmvel,projectVecAonUB(cmvel,(0.0, 1.0, 0.0)))
    Vvec = wvel
    if not self.inStrideTest() :
      torso.setLinearVel(Vvec)
      torso.setAngularVel(Wvec)
      
    # Lifting force generated by rotating foot at ankle --
    # applied thru c.g. to reduce moment on figure's frame
    r_foot = fig.r_foot
    l_foot = fig.l_foot
    if self.toesTouchingGround(r_foot) or \
       self.toesTouchingGround(l_foot)    :
      (gx, gy, gz) = fig.world.getGravity()
      Fvec = (0.0, -0.45*totmass*gy, 0.0)
    else :
      Fvec = (0.0, 0.0, 0.0)
    if not self.inStrideTest() :
      torso.addForceAtRelPos(Fvec,cgpos)
      
    if self.debug :
      print("Walk:  t=%8.4f" % (t))
      print("  zmp:   (%8.3f | %8.3f | %8.3f)" % zmp)
      print("  cmpos: (%8.3f | %8.3f | %8.3f)" % cmpos)
      print("  cmvel: (%8.3f | %8.3f | %8.3f)" % cmvel)
      print("  cmacc: (%8.3f | %8.3f | %8.3f)" % cmacc)
      print("  cgpos: (%8.3f | %8.3f | %8.3f)" % cgpos)
      print("  Vvec : (%8.3f | %8.3f | %8.3f)" % Vvec)
      print("  Wvec : (%8.3f | %8.3f | %8.3f)" % Wvec)
      print("  Fvec : (%8.3f | %8.3f | %8.3f)" % Fvec)  

    # Synchronize knee/foot motion with hip motion
    self.synchKneeFootwithHip(r_hip, rangv2, anglim, tstep)
    self.synchKneeFootwithHip(l_hip, langv2, anglim, tstep)
    
    if self.debug :
      print("right_leg : t=%8.4f, h_angv=%8.2f" % (t, rangv2*DPR))
      print("right_leg : t=%8.4f, h_ang=%8.2f, k_ang=%8.2f, a_ang=%8.2f" %\
            (t, fig.r_hip.motor.getAngle(2)*DPR,  \
                fig.r_knee.motor.getAngle(0)*DPR, \
                fig.r_ankle.motor.getAngle(0)*DPR ) )
      print("left_leg : t=%8.4f, h_angv=%8.2f" % (t, langv2*DPR))
      print("left_leg : t=%8.4f, h_ang=%8.2f, k_ang=%8.2f, a_ang=%8.2f" %\
            (t, fig.l_hip.motor.getAngle(2)*DPR,  \
                fig.l_knee.motor.getAngle(0)*DPR, \
                fig.l_ankle.motor.getAngle(0)*DPR ) )
                
    # Synchronize arm motion with hip motion
    self.synchArmMotionwithHip( fig.r_shoulder, \
                                langv2, anglim, tstep )
    self.synchArmMotionwithHip( fig.l_shoulder, \
                                rangv2, anglim, tstep )
                
    # Keep head looking forward
    n_joint = fig.neck.joint
    n_motor = fig.neck.motor
    angle   = -anglim/4.0
    rotateAxisToAngle(n_motor, 0, angle, TOL, RATE)
    rotateAxisAtRate(n_motor, 2, -wangv)
    
    if isinstance(n_joint, ode.BallJoint) :
      # Keep head from wobbling side to side
      rotateAxisToZero(n_motor, 1, TOL, RATE)
      
    # Bend forward slightly at waist
    w_joint = fig.waist.joint
    w_motor = fig.waist.motor
    angle   = anglim/4.0
    rotateAxisToAngle(w_motor, 0, angle, TOL, RATE)
    
    if isinstance(w_joint, ode.BallJoint) :
      # Keep waist from bending side to side
      rotateAxisToZero(w_motor, 1, TOL, RATE)
    
    # Rotate waist with hips
    wang = w_motor.getAngle(2)
    rotateAxisAtRate(w_motor, 2, wangv)
    if self.debug :
      print("waist : t=%8.4f, w_ang=%8.2f, w_angv=%8.2f" % \
            (t, wang*DPR, wangv*DPR) )
            
  def turnHeadTowardTarget(self, target, t, tstep):
  
    if not (self.figure.head and target) : return
    
    head = self.figure.head.body
  
    tpos = target.getPosition()
    hpos = head.getPosition()
    hang = head.getAngularVel()
    hvec = head.vectorToWorld( (0.0, 0.0, 1.0) )  # Eyes forward
    dvec = vecSub(tpos,hpos)
    dmag = vecMag(dvec)
    if dmag > 0.0 : dvec = vecMulS(dvec, 1.0/dmag)
    else          : dvec = hvec
    if acosVecDotP(hvec,dvec)*DPR < 60.0 : see = True
    else                                 : see = False
    nvec = vecCrossP(hvec,dvec)
    nmag = max(min(1.0,vecMag(nvec)),-1.0)
    if abs(nmag) > 0.0 :
      # turn head toward target
      unvec = unitVec(nvec)
      rate  = asin(nmag)/tstep - vecDotP(hang,unvec)
      nvec  = vecMulS(unvec, rate)    
      rvec  = head.vectorFromWorld(nvec)
      ar1   = vecDotP(rvec,(1.0,0.0,0.0))
      ar2   = vecDotP(rvec,(0.0,1.0,0.0))
      head.addRelTorque((0.1*ar1/tstep, 0.1*ar2/tstep, 0.0))
    
    return (see)
  
  def turnTorsoTowardTarget(self, target, t, tstep):
  
    if not (self.figure.torso and target) : return
    
    torso = self.figure.torso.body
  
    tpos = target.getPosition()
    bpos = torso.getPosition()
    bang = torso.getAngularVel()
    bvec = torso.vectorToWorld( (0.0, 0.0, 1.0) )
    dvec = vecSub(tpos,bpos)
    dmag = vecMag(dvec)
    if dmag > 0.0 : dvec = vecMulS(dvec, 1.0/dmag)
    else          : dvec = bvec
    if acosVecDotP(bvec,dvec)*DPR < 45.0 : facing = True
    else                                 : facing = False
    nvec = vecCrossP(bvec,dvec)  
    nmag = max(min(1.0,vecMag(nvec)),-1.0)
    if abs(nmag) > 0.0 :
      # turn torso toward target
      unvec = unitVec(nvec)
      rate  = asin(nmag)/tstep
      nvec  = vecMulS(unvec, rate)         # Rotation torso about
      rate  = vecDotP(nvec,(0.0,1.0,0.0))  # the world vertical 
      rate -= vecDotP(bang,(0.0,1.0,0.0))  # axis only
      torso.addTorque((0.0,0.25*rate/tstep,0.0))
    
    return (facing)
    
  def printAngularVelOfTargetWRTJoint(self, target, joint, t):
    
    # Get angular velocity of target WRT joint
    Wvec  = self.calcAngularVelAwrtB(target, joint.getBody(1))
    wvec  = joint.getBody(1).vectorFromWorld(Wvec)
    wmag  = vecMag(wvec)
    uwvec = unitVec(wvec)
    
    print("%s : t=%8.4f, wvec=%8.3f, " \
          "uwvec=(%8.3f | %8.3f | %8.3f)" % \
          (joint.label, t, wmag, uwvec[0], uwvec[1], uwvec[2]) )
          
    Wlos  = self.calcLosRateAwrtB(target, joint.getBody(1))
    wlos  = joint.getBody(1).vectorFromWorld(Wlos)
    wmag  = vecMag(wlos)
    uwlos = unitVec(wlos)
    
    print("%s : t=%8.4f, wlos=%8.3f, " \
          "uwlos=(%8.3f | %8.3f | %8.3f)" % \
          (joint.label, t, wmag, uwlos[0], uwlos[1], uwlos[2]) )
          
  def rotateWristTowardTarget(self, elbow, wrist, hand, \
                                    target, t, tstep):
    
    if not (wrist and hand and target) : return
      
    joint = wrist.joint
    motor = wrist.motor
    
    if not (joint and motor) : return
          
    if self.debug :
      self.printAngularVelOfTargetWRTJoint(target, joint, t)
    
    epos = elbow.body.getPosition()
    wpos = wrist.body.getPosition()
    hpos = hand.body.getPosition()
    tpos = target.getPosition()

    # Rotation normal vector and angle for hand toward target
    tvec = vecSub(tpos,wpos)
    hvec = vecSub(hpos,wpos)
    dlen = self.figure.wrist_radius + \
           self.figure.hand_length
    if (vecMag(tvec) - target.solid.radius) < dlen :
      # Palm of hand toward target
      uvec1 = unitVec(motor.getAxis(0))
      uvec2 = unitVec(hvec)
      pvec  = unitNormalVecFromAtoB(uvec1, uvec2)
      (nvec, ang) = unitNormalVecAndAngleFromAtoB(pvec,tvec)
    else : 
      # Finger tips in-line with fore_arm
      fvec = vecSub(wpos,epos)
      pvec = vecSub(hpos,wpos)
      (nvec, ang) = unitNormalVecAndAngleFromAtoB(pvec,fvec)

    # Calculate rotation rate (assuming pi rad/sec max rate)
    rate = min(max(-ang/tstep,-pi),pi)
    # Calculate required angular velocity
    Rvec = vecMulS(nvec, rate)
    # Get elbow joint motor axis in world space
    jaxis0 = motor.getAxis(0)
    # Apply required angular velocity about wrist axis,
    # accounting for existing fore_arm angular velocity
    angvjb1 = joint.getBody(0).getAngularVel()
    angv0   = vecDotP(Rvec, jaxis0) - \
              vecDotP(angvjb1, jaxis0)
    if self.debug :
      print("%s : t=%8.4f, ang=%7.2f, angv0=%8.3f" % \
            (joint.label, t, ang*DPR, angv0) )
          
    # Set wrist motor axis angular velocity
    rotateAxisAtRate(motor, 0, angv0)

  def rotateForeArmTowardTarget(self, elbow, wrist, hand, \
                                      target, t, tstep    ):
    
    if not (elbow and wrist and target) : return
      
    joint = elbow.joint
    motor = elbow.motor
    
    if not (joint and motor) : return
          
    if self.debug :
      self.printAngularVelOfTargetWRTJoint(target, joint, t)
      
    epos = elbow.body.getPosition()
    wpos = wrist.body.getPosition()
    hpos = hand.body.getPosition()
    tpos = target.getPosition()
              
    # Rotation normal vector and angle for fore_arm toward 
    # target
    tvec = vecSub(tpos,epos)
    dlen = self.figure.getElbowToWristLength() + \
           self.figure.wrist_radius            + \
           self.figure.hand_length
    if (vecMag(tvec)- target.solid.radius) > dlen :
      fvec = vecSub(wpos,epos)
      rot2 = False
    else : 
      fvec = vecSub(hpos,epos)
      rot2 = True
    (nvec, ang) = unitNormalVecAndAngleFromAtoB(fvec, tvec)
    
    # Calculate rotation rate (assuming pi rad/sec max rate)
    rate = min(max(-ang/tstep,-pi),pi)
    # Calculate required angular velocity
    Rvec = vecMulS(nvec,rate)
    """
    # Account for angular velocity of target wrt elbow
    Wvec = self.calcAngularVelAwrtB(target, joint.getBody(1))
    Rvec = vecAdd(Rvec,vecMulS(Wvec,-1.2))
    """
    # Account for line-of-sight rate of target wrt elbow
    Wlos = self.calcLosRateAwrtB(target, joint.getBody(1))
    Rvec = vecAdd(Rvec,vecMulS(Wlos,-1.2))
    
    # Rotation normal vector and angle for fore_arm axis 2 
    if rot2 :
      # Unit normal vector perpendicular to palm of hand
      hvec  = vecSub(hpos,wpos)
      uvec1 = unitVec(wrist.motor.getAxis(0))
      uvec2 = unitVec(hvec)
      pvec  = unitNormalVecFromAtoB(uvec1, uvec2)
      # Calculate rotation normal vector and angle for 
      # elbow axis 2
      tvec = vecSub(tpos,wpos)
      dlen = self.figure.wrist_radius + \
             self.figure.hand_length
      if (vecMag(tvec) - target.solid.radius) < dlen :
        # Rotate fore_arm so palm faces in direction of target
        wnvec   = unitNormalVecFromAtoB(pvec, tvec)
        maxis0  = motor.getAxis(0)
        maxis2  = motor.getAxis(2)
        envec01 = unitVec( vecSub( wnvec,                          \
                                   projectVecAonUB(wnvec,maxis2) ) )
        (nvec2, ang2) = unitNormalVecAndAngleFromAtoB(maxis0,envec01)
      else :
        # Rotate fore_arm so palm faces in direction of elbow 
        # motor axis 1
        maxis1 = motor.getAxis(1)
        (nvec2, ang2) = unitNormalVecAndAngleFromAtoB(pvec,maxis1)
    else :
      nvec2 = unitVec(motor.getAxis(2))
      ang2  = 0.0
      
    # Calculate rotation rate (assuming pi rad/sec max rate)
    rate2 = min(max(-ang2/tstep,-pi),pi)
    # Calculate required angular velocity
    Rvec2 = vecMulS(nvec2, rate2)
    
    # Get elbow joint motor axes in world space
    jaxis0 = motor.getAxis(0)
    jaxis2 = motor.getAxis(2)
    # Apply required angular velocity about elbow 
    # axes, accounting for existing upper_arm 
    # angular velocity
    angvjb1 = joint.getBody(0).getAngularVel()
    angv0   = vecDotP(Rvec, jaxis0) - \
              vecDotP(angvjb1, jaxis0)
    angv2   = vecDotP(Rvec2, jaxis2) - \
              vecDotP(angvjb1, jaxis2)
    if self.debug :
      print("%s : t=%8.4f, ang=%7.2f, " \
            "angv0=%8.3f, angv2=%8.3f" % \
            (joint.label, t, ang*DPR, angv0, angv2) )
            
    # Set elbow motor axes angular velocities
    rotateAxisAtRate(motor, 0, angv0)
    rotateAxisAtRate(motor, 2, angv2)
      
  def rotateUpperArmTowardTarget(self, shoulder, elbow, wrist, hand, \
                                       target, t, tstep              ):
      
    if not (shoulder and elbow and target) : return
      
    joint = shoulder.joint
    motor = shoulder.motor
    
    if not (joint and motor) : return
      
    if self.debug :
      self.printAngularVelOfTargetWRTJoint(target, joint, t)
      
    spos = shoulder.body.getPosition()
    epos = elbow.body.getPosition()
    wpos = wrist.body.getPosition()
    hpos = hand.body.getPosition()
    tpos = target.getPosition()
    
    # Rotation normal vector and angle at elbow for 
    # fore_arm to point toward target -- used to scale
    # shoulder rotations 
    tvec = vecSub(tpos,epos)
    fvec = vecSub(wpos,epos)
    (envec, eang) = unitNormalVecAndAngleFromAtoB(fvec,tvec)
    
    # Rotation normal vector and angle at elbow -- normal
    # vector perpendicular to plane formed by fore_arm and 
    # upper_arm, and parallel to direction of elbow hinge
    # joint when fore_arm is pointing toward target
    uvec  = vecSub(spos,epos)
    fvec  = vecSub(tpos,epos)
    envec = unitNormalVecFromAtoB(fvec,uvec)
    
    # Rotation normal vector and angle for shoulder axis 0;
    # moves upper_arm outward from torso, allowing fore_arm
    # to bend inward toward figure center-line
    maxis0  = motor.getAxis(0)
    maxis2  = motor.getAxis(2)
    snvec12 = unitVec( vecSub( envec,                          \
                               projectVecAonUB(envec,maxis0) ) )
    (nvec0, ang0) = unitNormalVecAndAngleFromAtoB(maxis2,snvec12)
    
    # Calculate rotation rate (assuming pi rad/sec max rate);
    # scale rate with ratio of shoulder rotation angle to
    # elbow rotation angle in order to sync the rotations
    sfac  = (1.0+abs(ang0))/(1.0+abs(eang))
    rate0 = min(max(ang0*sfac/tstep,-pi),pi)
    # Calculate required angular velocity
    Rvec0 = vecMulS(nvec0, rate0)
    
    # Predicted elbow position when fore_arm is 
    # pointing toward target
    dlen = self.figure.getElbowToWristLength() + \
           target.solid.radius
    dvec = vecMulS(unitVec(vecSub(epos,tpos)), dlen)
    ppos = vecAdd(tpos,dvec)
    
    # Rotation normal vector and angle for upper_arm toward 
    # elbow's predicted position from target when fore_arm
    # points toward target
    pvec = vecSub(ppos,spos)
    uvec = vecSub(epos,spos)
    (nvec, ang) = unitNormalVecAndAngleFromAtoB(uvec, pvec)

    # Calculate rotation rate (assuming pi rad/sec max rate);
    # scale rate with ratio of shoulder rotation angle to
    # elbow rotation angle in order to sync the rotations
    sfac = (1.0+abs(ang))/(1.0+abs(eang))
    rate = min(max(-ang*sfac/tstep,-pi),pi)
    # Calculate required angular velocity
    Rvec = projectVecAonUB(vecMulS(nvec,rate), envec)
    """
    # Account for angular velocity of target wrt shoulder
    Wvec = self.calcAngularVelAwrtB(target, joint.getBody(1))
    Rvec = vecAdd(Rvec,vecMulS(Wvec,-1.2))
    """
    # Account for line-of-sight rate of target wrt shoulder
    Wlos = self.calcLosRateAwrtB(target, joint.getBody(1))
    Rvec = vecAdd(Rvec,vecMulS(Wlos,-1.2))
    
    if isinstance(joint, ode.UniversalJoint) :
      # Get shoulder joint motor axes in world space
      jaxis0 = motor.getAxis(0)
      jaxis2 = motor.getAxis(2)
      # Apply required angular velocities about shoulder 
      # axes, accounting for existing torso angular velocity
      angvjb1 = joint.getBody(0).getAngularVel()
      angv0   = vecDotP(Rvec0, jaxis0) - \
                vecDotP(angvjb1, jaxis0)
      angv2   = vecDotP(Rvec, jaxis2) - \
                vecDotP(angvjb1, jaxis2)
      if self.debug:
        print("%s : t=%8.4f, ang=%7.2f, " \
              "angv0=%8.3f, angv2=%8.3f" % \
              (joint.label, t, ang*DPR, angv0, angv2) )
            
      # Set shoulder motor axes angular velocities
      rotateAxisAtRate(motor, 0, angv0)
      rotateAxisAtRate(motor, 2, angv2)

    elif isinstance(joint, ode.BallJoint) :
      # Get shoulder joint motor axes in world space
      jaxis0 = motor.getAxis(0)
      jaxis1 = motor.getAxis(1)
      jaxis2 = motor.getAxis(2)
      # Apply required angular velocities about shoulder
      # axes, accounting for existing torso angular velocity
      angvjb1 = joint.getBody(0).getAngularVel()
      angv0   = vecDotP(Rvec0, jaxis0) - \
                vecDotP(angvjb1, jaxis0)
      angv1   = vecDotP(Rvec, jaxis1) - \
                vecDotP(angvjb1, jaxis1)
      angv2   = vecDotP(Rvec, jaxis2) - \
                vecDotP(angvjb1, jaxis2)
      if self.debug :
        print("%s : t=%8.4f, ang=%7.2f, " \
              "angv0=%8.3f, angv1=%8.3f, angv2=%8.3f" % \
              (joint.label, t, ang*DPR, angv0, angv1, angv2) )
            
      # Convert angular velocity vector to Euler angle rates
      (ar0, ar1, ar2) = \
        getEulerAngRatesFromAngVelVec(motor,(angv0, angv1, angv2))
      if self.debug :
        print("%s : t=%8.4f, " \
              "ar0=%8.3f, ar1=%8.3f, ar2=%8.3f" % \
              (joint.label, t, ar0, ar1, ar2) )
            
      # Set shoulder motor axes angular velocities
      rotateAxisAtRate(motor, 0, ar0)
      rotateAxisAtRate(motor, 1, ar1)
      rotateAxisAtRate(motor, 2, ar2)
            
  def rotateArmTowardTarget(self, shoulder, target, t, tstep):
     
    if shoulder == self.figure.r_shoulder :
      elbow = self.figure.r_elbow
      wrist = self.figure.r_wrist
      hand  = self.figure.r_hand
    else :
      elbow = self.figure.l_elbow
      wrist = self.figure.l_wrist
      hand  = self.figure.l_hand
      
    self.rotateUpperArmTowardTarget( shoulder, elbow, wrist, hand, \
                                     target, t, tstep              )
                                     
    self.rotateForeArmTowardTarget( elbow, wrist, hand, \
                                    target, t, tstep    )
                                    
    self.rotateWristTowardTarget( elbow, wrist, hand, \
                                  target, t, tstep    )
                                          
  def reachHandTowardTarget(self, shoulder, target, t, tstep):
  
    if not (self.frame and target) : return
    
    self.rotateArmTowardTarget(shoulder, target, t, tstep)
      
  def reachHandsTowardTarget(self, target, t, tstep):
  
    if not (self.frame and target) : return
    
    r_hand  = self.figure.r_hand.body
    l_hand  = self.figure.l_hand.body

    armmass = self.figure.getArmMass()
    maxdmag = 0
    
    tpos = target.getPosition()

    if r_hand :          
      self.rotateArmTowardTarget( self.figure.r_shoulder, \
                                      target, t, tstep    )
      (sx, sy, sz) = r_hand.solid.boxsize
      rpos = r_hand.getRelPointPos( (0.0,-sy/2.0,0.0) ) # Finger tips
      dvec = vecSub(tpos,rpos)
      dmag = vecMag(dvec)
      uvec = unitVec(dvec)
      fmag = 0.5*dmag*armmass*9.81
      fvec = vecMulS(uvec, fmag)
      # r_hand.addForceAtRelPos(fvec, (0.0,-sy/2.0,0.0))
      if dmag > maxdmag : maxdmag = dmag
        
    if l_hand :
      self.rotateArmTowardTarget( self.figure.l_shoulder, \
                                  target, t, tstep        )
      (sx, sy, sz) = l_hand.solid.boxsize
      lpos = l_hand.getRelPointPos( (0.0,-sy/2.0,0.0) ) # Finger tips
      dvec = vecSub(tpos,lpos)
      dmag = vecMag(dvec)
      uvec = unitVec(dvec)
      fmag = 0.5*dmag*armmass*9.81
      fvec = vecMulS(uvec, fmag)
      # l_hand.addForceAtRelPos(fvec, (0.0,-sy/2.0,0.0))
      if dmag > maxdmag : maxdmag = dmag
    
    darm = self.figure.getExtendedArmLength()
    if (maxdmag > darm) and 0:
      r_foot  = self.figure.r_foot
      r_ankle = self.figure.r_ankle
      r_ball  = self.figure.r_ball
      self.pushFootAgainstGround(r_foot, r_ankle, r_ball, pi/4.0, tstep)
      l_foot  = self.figure.l_foot
      l_ankle = self.figure.l_ankle
      l_ball  = self.figure.l_ball
      self.pushFootAgainstGround(l_foot, l_ankle, l_ball, pi/4.0, tstep)
      # Apply jumping force up through c.g.
      totmass = self.figure.getTotMass()
      cmpos   = self.figure.calcCenterOfMass()
      torso   = self.figure.torso.body
      tpos    = torso.getPosition()
      cgpos   = vecSub(cmpos,tpos)
      (gx, gy, gz) = self.figure.world.getGravity()
      fmag = 0.45*totmass*gy
      fvec = (0.0, -fmag, 0.0)
      torso.addForceAtRelPos(fvec,cgpos)
       
  def kickFootTowardTarget(self, foot, target, t, tstep):
  
    if not (self.frame and target) : return
    
    if foot == self.figure.r_foot :
      k_hip   = self.figure.r_hip
      k_foot  = self.figure.r_foot
      k_ankle = self.figure.r_ankle
      p_foot  = self.figure.l_foot
      p_ankle = self.figure.l_ankle
      p_ball  = self.figure.l_ball
    elif foot == self.figure.l_foot :
      k_hip   = self.figure.l_hip
      k_foot  = self.figure.l_foot
      k_ankle = self.figure.l_ankle
      p_foot  = self.figure.r_foot
      p_ankle = self.figure.r_ankle
      p_ball  = self.figure.r_ball
    else:
      return

    totmass = self.figure.getTotMass()
  
    tpos = target.getPosition()
    
    # Kick out with this foot
    (sx, sy, sz) = k_foot.boxsize
    rpos = k_foot.body.getRelPointPos( (0.0,0.0,sz/2.0) ) # toes
    rvec = vecSub(tpos,rpos)
    rmag = vecMag(rvec)
    uvec = unitVec(rvec)
    fmag = 0.5*rmag*totmass*9.81
    fvec = vecMulS(uvec, fmag)
    k_foot.body.addForceAtRelPos(fvec, (0.0,0.0,sz/2.0))
      
    # Check if kicking foot will reach target
    bpos = k_hip.body.getPosition()
    dvec = vecSub(tpos,bpos)
    dmag = vecMag(dvec)
    dleg = self.figure.getExtendedLegLength()
    if dmag > dleg :
      # Push down with this foot
      if self.pushFootAgainstGround(p_foot, p_ankle, p_ball, pi/4.0, tstep) :
        # Apply jumping force up through c.g.
        cmpos = self.figure.calcCenterOfMass()
        torso = self.figure.torso.body
        tpos  = torso.getPosition()
        cgpos = vecSub(cmpos,tpos)
        (gx, gy, gz) = self.figure.world.getGravity()
        fmag = 0.45*totmass*gy
        fvec = (0.0, -fmag, 0.0)
        torso.addForceAtRelPos(fvec,cgpos)
