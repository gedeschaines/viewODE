# Control.py
#
# viewODE humanoid robotic figure control definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Disclaimer
#
#   See the file DISCLAIMER-GaryDeschaines

from __future__ import print_function
from sys import exit, modules, stderr

#
# Import ODE module for joint and motor modeling.

try:
    import ode
except ModuleNotFoundError or ImportError as ee:
    print("Error: PyODE not installed properly !! - {0}".format(ee.msg), file=stderr)
    exit(1)

#
# Import viewODE modules for humanoid figure joints, motors and vector math.

try:
    from Joints import *
    from Motors import *
    from vecMath import *
except ModuleNotFoundError or ImportError as ee:
    print("Error: {0}".format(ee.msg), file=stderr)

if not (modules["Joints"] and modules["Motors"] and modules["vecMath"]):
    print("Error: viewODE not installed properly !!", file=stderr)
    exit(1)


class Control:

    def __init__(self, figure):
        """ Constructor.
            figure -- viewODE figure
                    
            Initialize the viewODE figure control.
        """
        self.figure = figure
        self.frame = figure.frame
        self.debug = False

        self.torquemode = 1
        self.torqueaxis = 2
        self.rotzeroerr = True
        self.rotdamping = False
        self.rotlimited = False
        self.tfbdamping = False
        self.maintainjr = False
        self.direction = 1
        self.angerr_tol = 1.0 * RPD
        self.angvel_max = TWOPI
        self.manip_joint = None

    def debugOn(self):
        self.debug = True

    def debugOff(self):
        self.debug = False

    def toggleDebug(self):
        self.debug = not self.debug

    def resetConfig(self):

        self.torquemode = 1
        self.torqueaxis = 2
        self.rotzeroerr = True
        self.rotdamping = False
        self.rotlimited = False
        self.tfbdamping = False
        self.maintainjr = False
        self.direction = 1
        self.manip_joint = None

    def printConfig(self):

        if self.torquemode == 0:
            print("Torque Mode OFF")
            if self.rotdamping:
                print("Rotational damping ON")
            else:
                print("Rotational damping OFF")
        if self.torquemode == 1:
            print("Torque Mode ON (Restoring and Forcing/Limiting)")
            if self.rotzeroerr:
                print("Zero error restoring ON")
            else:
                print("Zero error restoring OFF")
            if self.rotdamping:
                print("Rotational damping ON")
            else:
                print("Rotational damping OFF")
            if self.rotdamping:
                if self.tfbdamping:
                    print("Torque feedback rotational damping ON")
                else:
                    print("Torque feedback rotational damping OFF")
            print("Forcing Torque Axis ", self.torqueaxis,
                  " Direction ", ("+" if self.direction > 0 else "-"))
            if self.rotlimited:
                print("Non-Forcing Axis Rotation Error Limiting ON")
            else:
                print("Non-Forcing Axis Rotation Error Limiting OFF")
            if self.maintainjr:
                print("Maintain manipulated joint rotation ON")
            else:
                print("Maintain manipulated joint rotation OFF")

        return

    def setTorqueModeOn(self):
        self.torquemode = 1

    def setTorqueModeOff(self):
        self.torquemode = 0

    def setZeroErrorOn(self):
        if not self.rotzeroerr:
            self.rotzeroerr = True
            print("Zero error restoring ON")

    def setZeroErrorOff(self):
        if self.rotzeroerr:
            self.rotzeroerr = False
            print("Zero error restoring OFF")

    def setTorqueControl(self, key):

        is_set = False

        if key == '0':
            is_set = True
            self.torquemode = not self.torquemode
            if self.torquemode:
                print("Torque Mode ON")
            else:
                print("Torque Mode OFF")
        elif key == '1':
            is_set = True
            self.torquemode = 1
            self.torqueaxis = 0
            print("Forcing Torque Axis 0")
        elif key == '2':
            is_set = True
            self.torquemode = 1
            self.torqueaxis = 1
            print("Forcing Torque Axis 1")
        elif key == '3':
            is_set = True
            self.torquemode = 1
            self.torqueaxis = 2
            print("Forcing Torque Axis 2")
        elif key == '+':
            is_set = True
            self.direction = 1
            print("Forcing Torque Direction +")
        elif key == '-':
            is_set = True
            self.direction = -1
            print("Forcing Torque Direction -")
        elif key == 'd' or key == 'D':
            # Toggle joint rotation damping
            is_set = True
            self.rotdamping = not self.rotdamping
            if self.rotdamping:
                print("Rotational Damping ON")
            else:
                print("Rotational Damping OFF")
        elif key == 'e' or key == 'E':
            # Toggle zero error restoring
            is_set = True
            self.rotzeroerr = not self.rotzeroerr
            if self.rotzeroerr:
                print("Zero error restoring ON")
            else:
                print("Zero error restoring OFF")
        elif key == 'l' or key == 'L':
            # Toggle rotation limited to torqued axis
            is_set = True
            self.rotlimited = not self.rotlimited
            if self.rotlimited:
                print("Non-Forcing Axis Rotation Error Limiting ON")
            else:
                print("Non-Forcing Axis Rotation Error Limiting OFF")
        elif key == 'm' or key == 'M':
            # Toggle maintain manipulated joint rotation
            is_set = True
            self.maintainjr = not self.maintainjr
            if self.maintainjr:
                print("Maintain manipulated joint rotation ON")
            else:
                print("Maintain manipulated joint rotation OFF")
        elif key == 't' or key == 'T':
            # Toggle torque feedback rotational damping
            is_set = True
            self.tfbdamping = not self.tfbdamping
            if self.tfbdamping:
                print("Torque feedback rotational damping ON")
                self.rotdamping = True
                print("Rotational damping ON")
                self.frame.setFeedbackOn()
                print("Feedback ON")
            else:
                print("Torque feedback rotational damping OFF")

        return is_set

    def printFeedback(self, name, t, p, fb):

        F1 = vecMag(fb[0])
        T1 = vecMag(fb[1])
        F2 = vecMag(fb[2])
        T2 = vecMag(fb[3])

        print("Feedback:  %s, t=%8.4f, p=%1d" % (name, t, p))
        print("  F1:  %8.3f (%8.3f | %8.3f | %8.3f)" %
              (F1, fb[0][0], fb[0][1], fb[0][2]))
        print("  T1:  %8.3f (%8.3f | %8.3f | %8.3f)" %
              (T1, fb[1][0], fb[1][1], fb[1][2]))
        print("  F2:  %8.3f (%8.3f | %8.3f | %8.3f)" %
              (F2, fb[2][0], fb[2][1], fb[2][2]))
        print("  T2:  %8.3f (%8.3f | %8.3f | %8.3f)" %
              (T2, fb[3][0], fb[3][1], fb[3][2]))
        print("%s : t=%8.4f, p=%1d, F1=%10.3f, T1=%10.3f" %
              (name, t, p, F1, T1))
        print("%s : t=%8.4f, p=%1d, F2=%10.3f, T2=%10.3f" %
              (name, t, p, F2, T2))

    def setManipJoint(self, joint):
        self.manip_joint = joint

    def applyJointDamping(self, t, tstep):

        if not self.frame:
            return

        tfb = self.tfbdamping
        show = self.debug

        for j in self.frame.joints:
            name = j.label
            pick = j.getBody(1).solid.wire
            fb = None
            if show:
                fb = j.getFeedback()
                if fb:
                    self.printFeedback(name, t, pick, fb)
            if isinstance(j, ode.HingeJoint):
                if show:
                    ang = j.getAngle()
                    ar = j.getAngleRate()
                    print("%s : t=%8.4f, p=%1d, ang=%6.3f, ar=%8.3f" %
                          (name, t, pick, ang, ar))
                if self.rotdamping:
                    applyHingeJointDamping(show, t, pick, j, tfb, fb)
                else:
                    j.setParam(ode.ParamFMax, 0.0)
            elif isinstance(j, ode.UniversalJoint):
                if show:
                    ang1 = j.getAngle1()
                    ang2 = j.getAngle2()
                    ar1 = j.getAngle1Rate()
                    ar2 = j.getAngle2Rate()
                    print("%s : t=%8.4f, p=%1d, ang1=%6.3f, ang2=%6.3f, ar1=%8.3f, ar2=%8.3f" %
                          (name, t, pick, ang1, ang2, ar1, ar2))
                if self.rotdamping:
                    applyUniversalJointDamping(show, t, pick, j, tfb, fb)
                else:
                    j.setParam(ode.ParamFMax, 0.0)
                    j.setParam(ode.ParamFMax2, 0.0)
            elif isinstance(j, ode.BallJoint):
                if show:
                    motor = j.getBody(1).solid.motor
                    if motor:
                        ang0 = motor.getAngle(0)
                        ang1 = motor.getAngle(1)
                        ang2 = motor.getAngle(2)
                        print("%s : t=%8.4f, p=%1d, ang0=%6.3f, ang1=%6.3f, ang2=%6.3f" %
                              (name, t, pick, ang0, ang1, ang2))
                    (ar0, ar1, ar2) = getBallJointAngularRates(j)
                    print("%s : t=%8.4f, p=%1d, ar0=%8.3f, ar1=%8.3f, ar2=%8.3f" %
                          (name, t, pick, ar0, ar1, ar2))
                if self.rotdamping:
                    applyBallJointDamping(show, t, pick, j, tfb, fb)
                else:
                    j.setParam(ode.ParamFMax, 0.0)
                    j.setParam(ode.ParamFMax2, 0.0)
                    j.setParam(ode.ParamFMax3, 0.0)

    def applyRestoringTorque(self, t, tstep, m):

        name = m.label
        pick = m.getBody(1).solid.wire
        show = self.debug

        TOL = self.angerr_tol
        RATE = self.angvel_max

        for k in range(m.getNumAxes()):
            loStop = m.getParam(AMotorAxisParams[k]['LoStop'])
            hiStop = m.getParam(AMotorAxisParams[k]['HiStop'])
            if hiStop > loStop:
                # This motor axis can rotate
                error = m.getAngle(k)
                if abs(error) > TOL:
                    rotateAxisToZero(m, k, TOL, RATE)
                    if show:
                        print("%s : t=%8.4f, p=%1d, error=%6.2f - axis %d restoring" %
                              (name, t, pick, error, k))
                else:
                    removeTorqueFromAxis(m, k)

    def applyForcingTorque(self, t, tstep, m):

        name = m.label
        pick = m.getBody(1).solid.wire
        show = self.debug

        TOL = self.angerr_tol
        RATE = 0.5 * self.angvel_max

        for k in range(m.getNumAxes()):
            loStop = m.getParam(AMotorAxisParams[k]['LoStop'])
            hiStop = m.getParam(AMotorAxisParams[k]['HiStop'])
            if hiStop > loStop:
                # This motor axis can rotate
                if k == self.torqueaxis:
                    angle = m.getAngle(k)
                    if (angle > (loStop - self.angerr_tol)) and \
                            (angle < (hiStop + self.angerr_tol)):
                        rotateAxisAtRate(m, k, self.direction * RATE)
                        if show:
                            print("%s : t=%8.4f, p=%1d, angle=%6.2f - axis %d forcing" %
                                  (name, t, pick, angle, k))
                    else:
                        removeTorqueFromAxis(m, k)
                elif self.rotlimited:
                    error = m.getAngle(k)
                    if abs(error) > TOL:
                        rotateAxisToZero(m, k, TOL, RATE)
                        if show:
                            print("%s : t=%8.4f, p=%1d, error=%6.2f - axis %d limiting" %
                                  (name, t, pick, error, k))
                    else:
                        removeTorqueFromAxis(m, k)

    def applyMotorTorques(self, t, tstep):

        if not self.frame:
            return

        show = self.debug

        for m in self.frame.motors:
            name = m.label
            pick = m.getBody(1).solid.wire
            fb = None
            if show:
                fb = m.getFeedback()
                if fb:
                    self.printFeedback(name, t, pick, fb)
            if isinstance(m, ode.AMotor):
                if show:
                    ang0 = m.getAngle(0)
                    ang1 = m.getAngle(1)
                    ang2 = m.getAngle(2)
                    print("%s : t=%8.4f, p=%1d, ang0=%6.3f, ang1=%6.3f, ang2=%6.3f" %
                          (name, t, pick, ang0, ang1, ang2))
                if self.torquemode == 0:
                    # Remove all motor restoring torques
                    if m.getMode() == ode.AMotorEuler:
                        removeTorqueFromAxis(m, m.getNumAxes())
                if self.torquemode == 1:
                    # Apply forcing, limiting or restoring torques
                    if m.getMode() == ode.AMotorEuler:
                        if m.getBody(1).solid.wire:
                            # Joint selected - Apply forcing/limiting torques
                            self.applyForcingTorque(t, tstep, m)
                        elif self.frame.figure.currStateJointManip():
                            # Manipulating joints, but joint not currently selected
                            if not self.maintainjr:
                                # Not maintaining manipulated rotation - Remove all
                                # forcing/limiting torques
                                removeTorqueFromAxis(m, m.getNumAxes())
                            if self.rotzeroerr:
                                # Joint not selected - Apply zero error restoring torques
                                self.applyRestoringTorque(t, tstep, m)
                        elif self.rotzeroerr:
                            # Joint not selected - Apply zero error restoring torques
                            self.applyRestoringTorque(t, tstep, m)
