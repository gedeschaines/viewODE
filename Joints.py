# PyODE Joints.py:  ODE figure frame joints definition module.

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
  from Motors import *
except:
  print("Error: Motors not installed properly !!")
try:
  from vecMath import *
except:
  print("Error: vecMath not installed properly !!")

def modeFromSpecs(specs,side,id):
    
  ( axes ) = specs['Axes'][str.join('-',(side,str(id)))]
    
  return ( axes['mode'] )
  
def axisFromSpecs(specs,side,id):
    
  ( axes ) = specs['Axes'][str.join('-',(side,str(id)))]
    
  return ( axes['axis'] )
  
def getJointSpecAxesInWorld(joint):
    
  # Transform joint spec axes to world space coordinates
  jaxis = []
  for k in range(3) :
    m    = modeFromSpecs(joint.specs,joint.side,k)
    axis = axisFromSpecs(joint.specs,joint.side,k)
    jaxis.append( unitVec( joint.getBody(m-1).vectorToWorld(axis) ) )
    
  return (jaxis[0], jaxis[1], jaxis[2])
    
def jointLabel(joint):
    
  b1name = joint.getBody(0).solid.label
  b2name = joint.getBody(1).solid.label
  jtype  = joint.type
  label  = b1name + "-" + str.join('',(b2name,jtype))
    
  return (label)
  
def printBodyJointTorqueInfo(case, solid, t, pick):
  
  joint = solid.joint
  if joint :
    name  = solid.label
    body  = solid.body
    Tbvec = body.vectorFromWorld(body.getTorque())
    Tb    = vecMag(Tbvec)
    fb    = joint.getFeedback()
    if fb :
      T1jvec = joint.getBody(0).vectorFromWorld(fb[1])
      T1j    = vecMag(T1jvec)
      T2jvec = joint.getBody(1).vectorFromWorld(fb[3])
      T2j    = vecMag(T2jvec)
      print("%s:  %s, t=%8.4f, p=%1d" % (case, name, t, pick) )
      print("  Tb,   Tbvec:  %9.2f (%9.2f | %9.2f | %9.2f)" % \
              (Tb, Tbvec[0],Tbvec[1],Tbvec[2]) )
      print("  T1j, T1jvec:  %9.2f (%9.2f | %9.2f | %9.2f)" % \
              (T1j, T1jvec[0],T1jvec[1],T1jvec[2]) )
      print("  T2j, T2jvec:  %9.2f (%9.2f | %9.2f | %9.2f)" % \
              (T2j, T2jvec[0],T2jvec[1],T2jvec[2]) )
    
def attachBallJoint(frame, side, body1, body2, specs):
    
  joint = ode.BallJoint(frame.world, frame.jointgroup)
  joint.attach(body1, body2)
  (x, y, z) = body2.getPosition()
  if body2.solid.shape == "cone" : y += body2.solid.h/2.0
  joint.setAnchor( (x, y, z) )
  joint.setFeedback(frame.feedback)
  joint.type  = "Ball"
  joint.side  = side
  joint.specs = specs
  joint.label = jointLabel(joint)
    
  frame.joints.append(joint)
  body2.solid.joint = joint
    
  return (joint)
    
def attachHinge2Joint(frame,side,body1,body2,specs):
    
  axis1  = axisFromSpecs(specs,side,0)
  axis2  = axisFromSpecs(specs,side,2)
  LoStop = specs['LoStop']
  HiStop = specs['HiStop']
    
  joint = ode.Hinge2Joint(frame.world, frame.jointgroup)
  joint.attach(body1, body2)
  (x, y, z) = body2.getPosition()
  if body2.solid.shape == "cone" : y += body2.solid.h/2.0
  joint.setAnchor( (x, y, z) )
  joint.setAxis1( axis1 )
  joint.setAxis2( axis2 )
  joint.setFeedback(frame.feedback)
  joint.setParam(ode.ParamLoStop, LoStop[0])
  joint.setParam(ode.ParamHiStop, HiStop[0])
  joint.type  = "Hinge2"
  joint.side  = side
  joint.specs = specs
  joint.label = jointLabel(joint)
    
  frame.joints.append(joint)
  body2.solid.joint = joint
          
  return (joint)
   
def attachHingeJoint(frame, side, body1, body2, specs):
      
  axis   = axisFromSpecs(specs,side,0)
  LoStop = specs['LoStop']
  HiStop = specs['HiStop']
    
  joint = ode.HingeJoint(frame.world, frame.jointgroup)
  joint.attach(body1, body2)
  (x, y, z) = body2.getPosition()
  if body2.solid.shape == "cone" : y += body2.solid.h/2.0
  joint.setAnchor( (x, y, z) )
  joint.setAxis( axis )
  joint.setFeedback(frame.feedback)
  joint.setParam(ode.ParamLoStop, LoStop[0])
  joint.setParam(ode.ParamHiStop, HiStop[0])
  joint.type  = "Hinge"
  joint.side  = side
  joint.specs = specs
  joint.label = jointLabel(joint)
    
  frame.joints.append(joint)
  body2.solid.joint = joint
    
  return (joint)
    
def attachFixedJoint(frame, side, body1, body2):
      
  joint = ode.FixedJoint(frame.world, frame.jointgroup)
  joint.attach(body1, body2)
  joint.setFixed()
  joint.setFeedback(frame.feedback)
  joint.type  = "Fixed"
  joint.side  = side
  joint.specs = None
  joint.label = jointLabel(joint)
    
  frame.joints.append(joint)
        
  return (joint)
  
def applyBallJointDampingTorque(show, t, p, j, fb):
  
  # Get ball joint axes in joint body 2 coordinates
  J0axis = axisFromSpecs(j.specs,j.side,0)
  J1axis = axisFromSpecs(j.specs,j.side,1)
  J2axis = axisFromSpecs(j.specs,j.side,2)
    
  # Get torques the joint applies to each body from the
  # feedback array in joint body 2 coordinates
  T1body = j.getBody(1).vectorFromWorld( fb[1] )
  T2body = j.getBody(1).vectorFromWorld( fb[3] )
    
  # Get the body torque components along the ball
  # joint's axes
  T10d    = vecDotP(T1body, J0axis)
  T20d    = vecDotP(T2body, J0axis)
  T10dvec = vecMulS(J0axis, T10d)
  T20dvec = vecMulS(J0axis, T20d)
    
  T11d    = vecDotP(T1body, J1axis)
  T21d    = vecDotP(T2body, J1axis)
  T11dvec = vecMulS(J1axis, T11d)
  T21dvec = vecMulS(J1axis, T21d)
    
  T12d    = vecDotP(T1body, J2axis)
  T22d    = vecDotP(T2body, J2axis)
  T12dvec = vecMulS(J2axis, T12d)
  T22dvec = vecMulS(J2axis, T22d)
    
  if show :
    print("Joint:  %s, t=%8.4f, p=%1d" % (j.label, t, p) )
    print("  ball axis0:  (%8.3f | %8.3f | %8.3f)" % \
            (J0axis[0], J0axis[1], J0axis[2]) )
    print("  T11d, T11dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T10d, T10dvec[0], T10dvec[1], T10dvec[2]) )
    print("  T21d, T21dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T20d, T20dvec[0], T20dvec[1], T20dvec[2]) )
    print("  ball axis1:  (%8.3f | %8.3f | %8.3f)" % \
            (J1axis[0], J1axis[1], J1axis[2]) )
    print("  T11d, T11dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T11d, T11dvec[0], T11dvec[1], T11dvec[2]) )
    print("  T21d, T21dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T21d, T21dvec[0], T21dvec[1], T21dvec[2]) )
    print("  ball axis2:  (%8.3f | %8.3f | %8.3f)" % \
            (J2axis[0], J2axis[1], J2axis[2]) )
    print("  T12d, T12dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T12d, T12dvec[0], T12dvec[1], T12dvec[2]) )
    print("  T22d, T22dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T22d, T22dvec[0], T22dvec[1], T22dvec[2]) )
              
  # Apply limited joint damping torque
  motor = j.motor
  if motor : 
    maxis0 = j.getBody(0).vectorFromWorld(motor.getAxis(0))
    maxis1 = j.getBody(1).vectorFromWorld(motor.getAxis(1))
    maxis2 = j.getBody(1).vectorFromWorld(motor.getAxis(2))
    if show :
      print("Motor:  %s t=%8.4f p=%1d" % (motor.label, t, p) )
      print("  maxis0: (%8.3f | %8.3f | %8.3f)" % \
              (maxis0[0], maxis0[1], maxis0[2]) )
      print("  maxis1: (%8.3f | %8.3f | %8.3f)" % \
              (maxis1[0], maxis1[1], maxis1[2]) )
      print("  maxis2: (%8.3f | %8.3f | %8.3f)" % \
              (maxis2[0], maxis2[1], maxis2[2]) )
    #Use motor to apply damping torque
    Td0   = -j.specs['Dratio'][0]*(T10d + T20d)
    Td1   = -j.specs['Dratio'][1]*(T11d + T21d)
    Td2   = -j.specs['Dratio'][2]*(T12d + T22d)
    FMax0 =  j.specs['FMax'][0]
    FMax1 =  j.specs['FMax'][1]
    FMax2 =  j.specs['FMax'][2]
    if show :
      print("%s : t=%8.4f, p=%1d, Td0=%10.3f, Td1=%10.3f, Td2=%10.3f" % \
          (j.label, t, p, Td0, Td1, Td2) )
    motor.addTorques(Td0, Td1, Td2)
    motor.setParam(ode.ParamFMax,  FMax0)
    motor.setParam(ode.ParamFMax2, FMax1)
    motor.setParam(ode.ParamFMax3, FMax2)
    
def getBallJointAngularRates(j):

  b0angvel = j.getBody(0).getAngularVel()
  b1angvel = j.getBody(1).getAngularVel()
    
  angv = []
  for k in range(3) :
    jmode   = modeFromSpecs(j.specs,j.side,k)
    jaxis   = axisFromSpecs(j.specs,j.side,k)
    axis    = j.getBody(jmode-1).vectorToWorld(jaxis)
    angv.append(vecDotP(b0angvel,axis) - vecDotP(b1angvel,axis))
    
  (ar0, ar1, ar2) = getEulerAngRatesFromAngVelVec(j.motor, angv)
   
  return (ar0, ar1, ar2)
    
def applyBallJointDamping(show, t, p, j, tfb, fb):
  
  motor = j.motor
  
  if motor :
    #Use motor to apply damping torque

    if tfb :
      if not fb : fb = j.getFeedback()
      if fb :
        applyBallJointDampingTorque(show, t, p, j, fb)
    else :
      (ar0, ar1, ar2) = getBallJointAngularRates(j)
      Td0   = -j.specs['Dratio'][0]*ar0
      Td1   = -j.specs['Dratio'][1]*ar1
      Td2   = -j.specs['Dratio'][2]*ar2
      FMax0 =  j.specs['FMax'][0]
      FMax1 =  j.specs['FMax'][1]
      FMax2 =  j.specs['FMax'][2]
      if show :
        print("%s : t=%8.4f, p=%1d, Td0=%10.3f, Td1=%10.3f, Td2=%10.3f" % \
            (j.label, t, p, Td0, Td1, Td2) )
      motor.addTorques(Td0, Td1, Td2)
      motor.setParam(ode.ParamFMax,  FMax0)
      motor.setParam(ode.ParamFMax2, FMax1)
      motor.setParam(ode.ParamFMax3, FMax2)
      
def applyHinge2JointDampingTorque(show, t, p, j, fb):
    
  # Get joint hinge axes in joint body 2 coordinates
  H1axis = unitVec( j.getBody(1).vectorFromWorld( j.getAxis1() ) )
  H2axis = unitVec( j.getBody(1).vectorFromWorld( j.getAxis2() ) )

  # Get torques the joint applies to each body from the
  # feedback array in joint body 2 coordinates
  T1body = j.getBody(1).vectorFromWorld( fb[1] )
  T2body = j.getBody(1).vectorFromWorld( fb[3] )
    
  # Get the body torque components along the joint's
  # hinge axes 
  T11d    = vecDotP(T1body, H1axis)
  T21d    = vecDotP(T2body, H1axis)
  T11dvec = vecMulS(H1axis, T11d)
  T21dvec = vecMulS(H1axis, T21d)
    
  T12d    = vecDotP(T1body, H2axis)
  T22d    = vecDotP(T2body, H2axis)
  T12dvec = vecMulS(H2axis, T12d)
  T22dvec = vecMulS(H2axis, T22d)
    
  if show :
    print("Joint: %s, t=%8.4f, p=%1d" % (j.label, t, p) )
    print("  hinge axis1:  (%8.3f | %8.3f | %8.3f)" % \
            (H1axis[0], H1axis[1], H1axis[2]) )
    print("  T11d, T11dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T11d, T11dvec[0], T11dvec[1], T11dvec[2]) )
    print("  T21d, T21dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T21d, T21dvec[0], T21dvec[1], T21dvec[2]) )
    print("  hinge axis2:  (%8.3f | %8.3f | %8.3f)" % \
            (H2axis[0], H2axis[1], H2axis[2]) )
    print("  T12d, T12dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T12d, T12dvec[0], T12dvec[1], T12dvec[2]) )
    print("  T22d, T22dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T22d, T22dvec[0], T22dvec[1], T22dvec[2]) )
              
  # Apply limited joint damping torque
  Td1   = -j.specs['Dratio'][0]*(T11d + T21d)
  Td2   = -j.specs['Dratio'][2]*(T12d + T22d)
  FMax1 =  j.specs['FMax'][0]
  FMax2 =  j.specs['FMax'][2]
  if show :
    print("%s : t=%8.4f, p=%1d, Td1=%10.3f, Td2=%10.3f" % \
          (j.label, t, p, Td1, Td2) )
  j.addTorques(Td1, Td2)
  j.setParam(ode.ParamFMax,  FMax1)
  j.setParam(ode.ParamFMax2, FMax2)
    
def applyHinge2JointDamping(show, t, p, j, tfb, fb):
    
  if tfb :
    if not fb : fb = j.getFeedback()
    if fb :
      applyHinge2JointDampingTorque(show, t, p, j, fb)
  else :
    Td1   = -j.specs['Dratio'][0]*j.getAngle1Rate()
    Td2   = -j.specs['Dratio'][2]*j.getAngle2Rate()
    FMax1 =  j.specs['FMax'][0]
    FMax2 =  j.specs['FMax'][2]
    if show :
      print("%s : t=%8.4f, p=%1d, Td1=%10.3f, Td2=%10.3f" % \
            (j.label, t, p, Td1, Td2) )
    j.addTorques(Td1, Td2)
    j.setParam(ode.ParamFMax,  FMax1)
    j.setParam(ode.ParamFMax2, FMax2)
  
def applyHingeJointDampingTorque(show, t, p, j, fb):
    
  # Get joint hinge axis in joint body 2 coordinates
  Haxis = unitVec( j.getBody(1).vectorFromWorld( j.getAxis() ) )
    
  # Get torques the joint applies to each body from the
  # feedback array in joint body 2 coordinates
  T1body = j.getBody(1).vectorFromWorld( fb[1] )
  T2body = j.getBody(1).vectorFromWorld( fb[3] )
    
  # Get the body torque components along the joint's
  # hinge axis 
  T1d    = vecDotP(T1body,Haxis)
  T2d    = vecDotP(T2body,Haxis)
  T1dvec = vecMulS(Haxis, T1d)
  T2dvec = vecMulS(Haxis, T2d)
  if show :
    print("Joint:  %s, t=%8.4f, p=%1d" % (j.label, t, p) )
    print("  hinge axis:  (%8.3f | %8.3f | %8.3f)" % \
            (Haxis[0], Haxis[1], Haxis[2]) )
    print("  T1d, T1dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T1d, T1dvec[0], T1dvec[1], T1dvec[2]) )
    print("  T2d, T2dvec:  %8.3f (%8.3f | %8.3f | %8.3f)" % \
            (T2d, T2dvec[0], T2dvec[1], T2dvec[2]) )
    
  # Apply limited joint damping torque
  Td   = -j.specs['Dratio'][0]*(T1d + T2d)
  FMax =  j.specs['FMax'][0]
  if show :
    print("%s : t=%8.4f, p=%1d, Td=%10.3f" % \
          (j.label, t, p, Td) )
  j.addTorque(Td)
  j.setParam(ode.ParamFMax, FMax)
  
def applyHingeJointDamping(show, t, p, j, tfb, fb):
  
  if tfb :
    if not fb : fb = j.getFeedback()
    if fb :
      applyHingeJointDampingTorque(show, t, p, j, fb)
  else :
    Td   = -j.specs['Dratio'][0]*j.getAngleRate()
    FMax =  j.specs['FMax'][0]
    if show :
      print("%s : t=%8.4f, p=%1d, Td=%10.3f" % \
            (j.label, t, p, Td) )
    j.addTorque(Td)
    j.setParam(ode.ParamFMax, FMax)
