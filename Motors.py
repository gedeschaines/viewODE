# Motors.py
#
# viewODE figure frame motors definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Disclaimer
#
#   See the file DISCLAIMER-GaryDeschaines

from sys  import exit
from math import cos, sin

#
# Import ODE module for joint and motor models.

try:
  import ode
except:
  print("Error: This module requires PyODE !!")
  exit()

#
# Import viewODE module for vector math.

try:
  from vecMath import *
except:
  print("Error: vecMath not installed properly !!")
  
AMotorAxisParams = ( { 'LoStop' : ode.ParamLoStop,
                       'HiStop' : ode.ParamHiStop,
                       'Vel'    : ode.ParamVel,
                       'FMax'   : ode.ParamFMax },
                     { 'LoStop' : ode.ParamLoStop2,
                       'HiStop' : ode.ParamHiStop2,
                       'Vel'    : ode.ParamVel2,
                       'FMax'   : ode.ParamFMax2 },
                     { 'LoStop' : ode.ParamLoStop3,
                       'HiStop' : ode.ParamHiStop3,
                       'Vel'    : ode.ParamVel3,
                       'FMax'   : ode.ParamFMax3 } )

def axesFromSpecs(specs,side,num):
    
  axes = []
  for i in range(num):
    axes.append(specs['Axes'][str.join('-',(side,str(i)))])
      
  return ( axes )
    
def motorLabel(motor):
  """
  Constructs and returns the label string assembled from the
  labels for the given motor's joint body solids, joint type
  and motor type.

  @param motor: A viewODE figure frame motor.
  @type  motor: ODE motor object

  @return: motor label
  @rtype: string
  """
  b1name = motor.joint.getBody(0).solid.label
  b2name = motor.joint.getBody(1).solid.label
  jtype  = motor.joint.type
  mtype  = motor.type
  label  = b1name + "-" + str.join('',(b2name,jtype,mtype))
    
  return label
    
def attachBallAMotor(frame,joint,specs):
  """
  Instantiates an ODE AMotor object to attach between the given Ball
  joint's joined bodies as defined by the given figure joint specs,
  appends the motor to the given frame motors list, and links the
  motor to the given joint's 2nd body solid.

  @param frame: A viewODE figure frame for the instantiated motor.
  @type  frame: viewODE Frame object
  @param joint: A viewODE figure frame joint to be driven by the instantiated motor.
  @type  joint: ODE BallJoint object
  @param specs: Figure joint and motor specs for the given ball joint.
  @type  specs: viewODE Figure class JointSpecs dictionary entry for the given joint

  @return: The instantiated motor attached to the given ball joint.
  @rtype: ODE AMotor object
  """
    
  if ( joint.type != "Ball" ) :
    print("attachBallAmotor: Joint type %s not Ball!" % \
      (joint.type) )
    exit(1)
      
  Axes   = axesFromSpecs(joint.specs,joint.side,3)
  LoStop = joint.specs['LoStop']
  HiStop = joint.specs['HiStop']
    
  motor = ode.AMotor(frame.world, frame.motorgroup)
  motor.attach(joint.getBody(0), joint.getBody(1))
  motor.setMode(ode.AMotorEuler)
  motor.setAxis(0, Axes[0]['mode'], Axes[0]['axis'])
  motor.setAxis(1, Axes[1]['mode'], Axes[1]['axis'])
  motor.setAxis(2, Axes[2]['mode'], Axes[2]['axis'])
  motor.setFeedback(frame.feedback)
  motor.setParam(ode.ParamLoStop,  LoStop[0])
  motor.setParam(ode.ParamHiStop,  HiStop[0])
  motor.setParam(ode.ParamLoStop2, LoStop[1])
  motor.setParam(ode.ParamHiStop2, HiStop[1])
  motor.setParam(ode.ParamLoStop3, LoStop[2])
  motor.setParam(ode.ParamHiStop3, HiStop[2])
  motor.type  = "AMotor"
  motor.joint = joint
  motor.label = motorLabel(motor)
  
  frame.motors.append(motor)
  joint.motor = motor
  joint.getBody(1).solid.motor = motor
    
  return motor
  
def attachHinge2AMotor(frame,joint,specs):
  """
  Instantiates an ODE AMotor object to attach between the given Hinge2
  joint's joined bodies as defined by the given figure joint specs,
  appends the motor to the given frame motors list, and links the
  motor to the given joint's 2nd body solid.

  @param frame: A viewODE figure frame for the instantiated motor.
  @type  frame: viewODE Frame object
  @param joint: A viewODE figure frame joint to be driven by the instantiated motor.
  @type  joint: ODE Hinge2Joint object
  @param specs: Figure joint and motor specs for the given hinge2 joint.
  @type  specs: viewODE Figure class JointSpecs dictionary entry for the given joint

  @return: The instantiated motor attached to the given hinge2 joint.
  @rtype: ODE AMotor object
  """
  if ( joint.type != "Hinge2" ) :
    print("attachHinge2Amotor: Joint type %s not Hinge2!" % \
      (joint.type) )
    exit(1)
      
  Axes   = axesFromSpecs(joint.specs,joint.side,3)
  LoStop = joint.specs['LoStop']
  HiStop = joint.specs['HiStop']
    
  motor = ode.AMotor(frame.world, frame.motorgroup)
  motor.attach(joint.getBody(0), joint.getBody(1))
  motor.setMode(ode.AMotorEuler)
  motor.setAxis(0, Axes[0]['mode'], Axes[0]['axis'])
  motor.setAxis(1, Axes[1]['mode'], Axes[1]['axis'])
  motor.setAxis(2, Axes[2]['mode'], Axes[2]['axis'])
  motor.setFeedback(frame.feedback)
  motor.setParam(ode.ParamLoStop,  LoStop[0])
  motor.setParam(ode.ParamHiStop,  HiStop[0])
  motor.setParam(ode.ParamLoStop2, 0.0)
  motor.setParam(ode.ParamHiStop2, 0.0)
  motor.setParam(ode.ParamLoStop3, LoStop[2])
  motor.setParam(ode.ParamHiStop3, HiStop[2])
  motor.type  = "AMotor"
  motor.joint = joint
  motor.label = motorLabel(motor)
  
  frame.motors.append(motor)
  joint.motor = motor
  joint.getBody(1).solid.motor = motor
  
  return motor
    
def attachHingeAMotor(frame,joint,specs):
  """
  Instantiates an ODE AMotor object to attach between the given Hinge
  joint's joined bodies as defined by the given figure joint specs,
  appends the motor to the given frame motors list, and links the
  motor to the given joint's 2nd body solid.

  @param frame: A viewODE figure frame for the instantiated motor.
  @type  frame: viewODE Frame object
  @param joint: A viewODE figure frame joint to be driven by the instantiated motor.
  @type  joint: ODE HingeJoint object
  @param specs: Figure joint and motor specs for the given hinge joint.
  @type  specs: viewODE Figure class JointSpecs dictionary entry for the given joint

  @return: The instantiated motor attached to the given hinge joint.
  @rtype: ODE AMotor object
  """
  if ( joint.type != "Hinge" ) :
    print("attachHingeAmotor: Joint type %s not Hinge!" % \
      (joint.type) )
    exit(1)
    
  Axes   = axesFromSpecs(joint.specs,joint.side,3)
  LoStop = joint.specs['LoStop']
  HiStop = joint.specs['HiStop']
    
  motor = ode.AMotor(frame.world, frame.motorgroup)
  motor.attach(joint.getBody(0), joint.getBody(1))
  motor.setMode(ode.AMotorEuler)
  motor.setAxis(0, Axes[0]['mode'], Axes[0]['axis'])
  motor.setAxis(1, Axes[1]['mode'], Axes[1]['axis'])
  motor.setAxis(2, Axes[2]['mode'], Axes[2]['axis'])
  motor.setFeedback(frame.feedback)
  motor.setParam(ode.ParamLoStop,  LoStop[0])
  motor.setParam(ode.ParamHiStop,  HiStop[0])
  motor.setParam(ode.ParamLoStop2, 0.0)
  motor.setParam(ode.ParamHiStop2, 0.0)
  motor.setParam(ode.ParamLoStop3, 0.0)
  motor.setParam(ode.ParamHiStop3, 0.0)
  motor.type  = "AMotor"
  motor.joint = joint
  motor.label = motorLabel(motor)
  
  frame.motors.append(motor)
  joint.motor = motor
  joint.getBody(1).solid.motor = motor
  
  return motor
  
def getEulerAngRatesFromAngVelVec(motor, Wvec):
  """ Assumes ang1 never goes to +/- pi/2.0
  """
  w0 = Wvec[0]
  w1 = Wvec[1]
  w2 = Wvec[2]
    
  ang0 = motor.getAngle(0)
  ang1 = motor.getAngle(1)
  ang2 = motor.getAngle(2)
    
  ar0 = (w0*cos(ang2) - w1*sin(ang2))/cos(ang1)
  ar1 = w0*sin(ang2) + w1*cos(ang2)
  ar2 = w2 - ar0*sin(ang1)
    
  return (ar0, ar1, ar2)
  
def getAngVelVecFromEulerAngRates(motor, Rates):
  """ Assumes ang1 never goes to +/- pi/2.0
  """
  ar0 = Rates[0]
  ar1 = Rates[1]
  ar2 = Rates[2]
    
  ang0 = motor.getAngle(0)
  ang1 = motor.getAngle(1)
  ang2 = motor.getAngle(2)
    
  w0 = ar1*sin(ang2) + ar0*cos(ang2)*cos(ang1)
  w1 = ar1*cos(ang2) - ar0*sin(ang2)*cos(ang1)
  w2 = ar2 + ar0*sin(ang1)
    
  return (w0, w1, w2)
  
def removeTorqueFromAxis(motor, axis):

  motor.setParam(AMotorAxisParams[axis]['Vel'],  0.0)
  motor.setParam(AMotorAxisParams[axis]['FMax'], 0.0)
  
def rotateAxisAtRate(motor, axis, rate):
    
  ang = motor.getAngle(axis)
  
  if rate > 0.0 :
    angHi = motor.getParam(AMotorAxisParams[axis]['HiStop'])
    if ang < angHi :
      FMax = motor.joint.specs['AMFMax'][axis]
      motor.setParam(AMotorAxisParams[axis]['Vel'],  rate)
      motor.setParam(AMotorAxisParams[axis]['FMax'], FMax)
      
  if rate < 0.0 :
    angLo = motor.getParam(AMotorAxisParams[axis]['LoStop'])
    if ang > angLo :
      FMax = motor.joint.specs['AMFMax'][axis]
      motor.setParam(AMotorAxisParams[axis]['Vel'],  rate)
      motor.setParam(AMotorAxisParams[axis]['FMax'], FMax)
      
def rotateAxisToZero(motor, axis, tol, rate):
    
  err = motor.getAngle(axis)
  if abs(err) > tol :
    FMax = motor.joint.specs['AMFMax'][axis]
    motor.setParam(AMotorAxisParams[axis]['Vel'],  -err*rate)
    motor.setParam(AMotorAxisParams[axis]['FMax'], FMax)
      
def rotateAxisToAngle(motor, axis, ang, tol, rate):
    
  ang = max(ang, motor.getParam(AMotorAxisParams[axis]['LoStop']))
  ang = min(ang, motor.getParam(AMotorAxisParams[axis]['HiStop']))
  err = motor.getAngle(axis) - ang
  if abs(err) > tol :
    FMax = motor.joint.specs['AMFMax'][axis]
    motor.setParam(AMotorAxisParams[axis]['Vel'],  -err*rate)
    motor.setParam(AMotorAxisParams[axis]['FMax'], FMax)
      
def rotateAxisToStop(motor, axis, stop, tol, rate):
    
  err = motor.getAngle(axis)- motor.joint.specs[stop][axis] 
  if abs(err) > tol :
    FMax = motor.joint.specs['AMFMax'][axis]
    motor.setParam(AMotorAxisParams[axis]['Vel'],  -err*rate)
    motor.setParam(AMotorAxisParams[axis]['FMax'], FMax)
