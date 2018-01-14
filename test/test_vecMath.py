
from math    import *
from vecMath import *

posX    = (1.0, 0.0, 0.0)
numk    = 9
ang_max = 180.0
ang_del = 360.0/(numk - 1)

print "Test of acosUVecDotP:"

for k in range(numk) :
  ang  = ang_max - k*ang_del
  rad  = ang*RPD
  R    = unitVec((cos(rad), sin(rad), 0.0))
  acos = acosUVecDotP(posX,R)*DPR
  print "  +X axis . (cos(%6.1f), sin(%6.1f), 0.0) = %7.2f" % \
        (ang, ang, acos) 

print "Test of atanVecCrossP:"

for k in range(numk) :
  ang  = ang_max- k*ang_del
  rad  = ang*RPD
  R    = unitVec((2*cos(rad), 2*sin(rad), 0.0))
  atan = atanVecCrossP(posX,R)*DPR
  print "  +X axis x (2cos(%6.1f), 2sin(%6.1f), 0.0) = %7.2f" % \
        (ang, ang, atan) 

print "Test of unitNormalVecAndAngleFromAtoB:"

for k in range(numk) :
  ang  = ang_max - k*ang_del
  rad  = ang*RPD
  R    = unitVec((2*cos(rad), 2*sin(rad), 0.0))
  (uNvec, angle) = unitNormalVecAndAngleFromAtoB(posX,R)
  print "  +X axis x (2cos(%6.1f), 2sin(%6.1f), 0.0) = "\
        "(%5.2f, %5.2f, %5.2f), %7.2f" % \
        (ang, ang, uNvec[0], uNvec[1], uNvec[2], angle*DPR)

