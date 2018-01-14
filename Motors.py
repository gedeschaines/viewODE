# PyODE Motors.py:  ODE figure frame motors definition module.

# Originally by Gary Deschaines

import sys

from math   import *
from string import *
  
try:
  import ode
except:
  print("Error: This module requires PyODE !!")
  sys.exit()

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
    
  b1name = motor.joint.getBody(0).solid.label
  b2name = motor.joint.getBody(1).solid.label
  jtype  = motor.joint.type
  mtype  = motor.type
  label  = b1name + "-" + str.join('',(b2name,jtype,mtype))
    
  return (label)
    
def attachBallAMotor(frame,joint,specs):
    
  if ( joint.type != "Ball" ) :
    print("attachBallAmotor: Joint type %s not Ball!" % \
      (joint.type) )
    sys.exit(1)
      
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
    
  return (motor)
  
def attachHinge2AMotor(frame,joint,specs):
    
  if ( joint.type != "Hinge2" ) :
    print("attachHinge2Amotor: Joint type %s not Hinge2!" % \
      (joint.type) )
    sys.exit(1)
      
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
  
  return (motor)
    
def attachHingeAMotor(frame,joint,specs):
    
  if ( joint.type != "Hinge" ) :
    print("attachHingeAmotor: Joint type %s not Hinge!" % \
      (joint.type) )
    sys.exit(1)
    
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
  
  return (motor)
  
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
