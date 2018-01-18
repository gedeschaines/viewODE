# vecMath.py
#
# Vector math functions module.
#
# Originally by Gary Deschaines, 2009
#
# Attributions
#
# + Matt Heinzen for PyODE Ragdoll Physics Tutorial program ragdoll-pyode-tutorial.py
#   available at http://monsterden.net/software/ragdoll-pyode-tutorial which was used
#   as a basis for the vector math functions herein.
#
# Disclaimers
#
#   See the file DISCLAIMER-GaryDeschaines
#   See the file DISCLAIMER-MatHeinzen

from math import pi, sqrt, acos, atan2

# Math Conversion Factors and Constants

RPD      = pi/180.0
DPR      = 180.0/pi
TWOPI    = 2.0*pi
HALFPI   = pi/2.0
INFINITY = float("inf")

# Vector Math

def vecMulS(V,s):
  return (V[0]*s, V[1]*s, V[2]*s)
  
def vecDivS(V,s):
  if s == 0.0 : return (INFINITY, INFINITY, INFINITY)
  else        : return (float(V[0])/s, float(V[1])/s, float(V[2])/s)
  
def vecAdd(A,B):
  return (A[0]+B[0], A[1]+B[1], A[2]+B[2])
  
def vecSub(A,B):
  return (A[0]-B[0], A[1]-B[1], A[2]-B[2])
  
def vecMagSq(V) :
  return (V[0]**2 + V[1]**2 + V[2]**2)
  
def vecMag(V):
  magsq = vecMagSq(V)
  if magsq > 0.0 : return sqrt(magsq)
  else           : return 0.0
  
def unitVec(V):
  mag = vecMag(V)
  if mag > 0.0 : return vecMulS(V,1.0/mag)
  else         : return (0.0, 0.0, 0.0)

def vecDotP(A,B):
  return (A[0]*B[0] + A[1]*B[1] + A[2]*B[2])
  
def acosUVecDotP(UA,UB):
  cosang = vecDotP(UA,UB)
  if cosang   < -1.0 : return pi
  elif cosang >  1.0 : return 0.0
  else               : return acos(cosang)
    
def acosVecDotP(A,B):
  UA = unitVec(A)
  UB = unitVec(B)
  return acosUVecDotP(UA,UB)
  
def projectVecAonUB(A,UB):
  return vecMulS(UB,vecDotP(A,UB))
    
def rejectionVecAfromUB(A,UB):
  return vecSub(A,projectVecAonUB(A,UB))
  
def vecCrossP(A,B):
  return ( A[1]*B[2] - B[1]*A[2],
           A[2]*B[0] - B[2]*A[0],
           A[0]*B[1] - B[0]*A[1] )
           
def atanVecCrossP(A,B):
  x = vecDotP(A,B)
  y = vecMag(vecCrossP(A,B))
  return atan2(y, x)
  
def unitNormalVecFromAtoB(A,B):
  return unitVec(vecCrossP(A,B))
  
def unitNormalVecAndAngleFromAtoB(A,B):
  Nvec = vecCrossP(A,B)
  return ( unitVec(Nvec), atan2(vecMag(Nvec),vecDotP(A,B)) )
  