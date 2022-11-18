# Frame.py
#
# viewODE humanoid jointed rigid body figure frame definition module.
#
# Originally by Gary Deschaines, 2009.
#
# Attributions
#
# + Matt Heinzen for PyODE Ragdoll Physics Tutorial program ragdoll-pyode-tutorial.py
#   available at http://monsterden.net/software/ragdoll-pyode-tutorial which was used
#   as a basis for the humanoid figure frame herein.
#
# Disclaimers
#
#   See the file DISCLAIMER-GaryDeschaines
#   See the file DISCLAIMER-MatHeinzen

from __future__ import print_function
from sys import exit, modules, stderr

#
# Import ODE module for world, space, body and joint models.

try:
    import ode
except ModuleNotFoundError or ImportError as ee:
    print("Error: PyODE not installed properly !! - {0}".format(ee.msg), file=stderr)
    exit(1)

#
# Import viewODE modules for solids, joints and motors.

try:
    from Solids import *
    from Joints import *
    from Motors import *
except ModuleNotFoundError or ImportError as ee:
    print("Error: {0}".format(ee.msg), file=stderr)

if not (modules["Solids"] and modules["Joints"] and modules["Motors"]):
    print("Error: viewODE not installed properly !!", file=stderr)
    exit(1)


class Frame:

    def __init__(self, figure):
        """
        A viewODE Frame class constructor to instantiate a humanoid robotic
        figure's frame object, which is composed of ODE bodies, joints and
        motors.

        @param figure: A figure for which to define its frame.
        @type  figure: viewODE Figure object
        """
        self.figure = figure
        self.world = figure.world
        self.space = figure.space
        self.balljoints = 0
        self.slimbones = 0
        self.feedback = True
        self.jointgroup = ode.JointGroup()
        self.motorgroup = ode.JointGroup()
        self.solids = []
        self.joints = []
        self.motors = []

    def delete(self):

        space = self.space

        for s in self.solids:
            s.body.disable()
            space.remove(s.geom)
        self.solids = []

        self.setFeedbackOff()
        self.feedback = True  # default

        self.jointgroup.empty()
        self.joints = []

        self.motorgroup.empty()
        self.motors = []

        return

    def resetConfig(self):

        self.balljoints = 0
        self.slimbones = 0
        self.setFeedbackOn()

    def setConfig(self, key):

        is_set = False

        if key == 'b' or key == 'B':
            # Re-assemble object with ball joints
            is_set = True
            self.balljoints = not self.balljoints
            if self.balljoints:
                print("Re-assemble frame with ball joints ON\n")
            else:
                print("Re-assemble frame with ball joints OFF\n")
        elif key == 's' or key == 'S':
            # Toggle slimbones arm and leg body segments
            is_set = True
            self.slimbones = not self.slimbones
            if self.slimbones:
                print("Toggle arm and leg slimbones ON\n")
            else:
                print("Toggle arm and leg slimbones OFF\n")

        return is_set

    def printConfig(self):

        if self.balljoints:
            print("Ball Joints ON")
        else:
            print("Ball Joints OFF")
        if self.slimbones:
            print("Slim Bones ON")
        else:
            print("Slim Bones OFF")
        if self.feedback:
            print("Feedback ON")
        else:
            print("Feedback OFF")

    def setState(self, key):

        is_set = False

        if key == 'c' or key == 'C':
            # Collapse object
            is_set = True
            self.setFeedbackOff()
            for j in self.joints:
                j.getBody(1).solid.joint = None
            self.jointgroup.empty()
            self.joints = []
            for m in self.motors:
                m.getBody(1).solid.motor = None
            self.motorgroup.empty()
            self.motors = []
        elif key == 'f' or key == 'F':
            # Toggle joint and motor feedback
            is_set = True
            self.toggleFeedback()
            if self.feedback:
                print("Feedback ON")
            else:
                print("Feedback OFF")

        return is_set

    def setJointFeedback(self):
        for j in self.joints:
            j.setFeedback(self.feedback)

    def setMotorFeedback(self):
        for m in self.motors:
            m.setFeedback(self.feedback)

    def setFeedbackOn(self):
        self.feedback = True
        self.setJointFeedback()
        self.setMotorFeedback()

    def setFeedbackOff(self):
        self.feedback = False
        self.setJointFeedback()
        self.setMotorFeedback()

    def toggleFeedback(self):
        self.feedback = not self.feedback
        self.setJointFeedback()
        self.setMotorFeedback()

    def createFigureFrame(self, figure, px, py, pz):
        """
        Creates a figure's frame in its world space.

        @param figure: A figure for which to define a frame.
        @type  figure: viewODE Figure object
        @param px: ODE world space position x coordinate.
        @type  px: float
        @param py: ODE world space position y coordinate.
        @type  py: float
        @param pz: ODE world space position z coordinate.
        @type  pz: float

        @return: ((x,y,z)) -- position coordinates in world space for center
                 of the figure's head.
        @rtype: tuple
        """

        figure.origin = (px, py, pz)

        world = figure.world
        space = figure.space
        frame = figure.frame
        sfac = figure.sfac

        # Create right and left legs

        dx = figure.pelvis_xdim * sfac / 2.0 + \
             figure.hip_radius * sfac

        (rx, ry, rz, rhip) = self.createFigureLeg(figure,
                                                  'R',
                                                  px - dx, py, pz)

        (lx, ly, lz, lhip) = self.createFigureLeg(figure,
                                                  'L',
                                                  px + dx, py, pz)

        # Pelvis

        h_tot = ry
        h = figure.pelvis_ydim * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz
        solid = solidBox(world, space,
                         "pelvis",
                         figure.pelvis_density,
                         figure.pelvis_xdim * sfac,
                         figure.pelvis_ydim * sfac,
                         figure.pelvis_zdim * sfac)

        frame.solids.append(solid)

        figure.pelvis = solid

        pelvis = solid.body
        pelvis.setPosition((x, y, z))

        # Attach hip joints and motors

        specs = figure.JointSpecs['hip']

        if frame.balljoints:
            joint = attachBallJoint(frame, 'R', pelvis, rhip, specs)
            motor = attachBallAMotor(frame, joint, specs)
            joint = attachBallJoint(frame, 'L', pelvis, lhip, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, 'R', pelvis, rhip, specs)
            motor = attachUniversalAMotor(frame, joint, specs)
            joint = attachUniversalJoint(frame, 'L', pelvis, lhip, specs)
            motor = attachUniversalAMotor(frame, joint, specs)

        # Waist joint

        h_tot += h
        h = figure.waist_height * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz
        """
        solid  = solidCone( world, space,                   \
                            "waist",                        \
                            figure.joint_density,           \
                            figure.waist_radius*sfac,       \
                            (figure.waist_radius/2.0)*sfac, \
                            figure.waist_height*sfac        )
        """
        solid = solidBall(world, space,
                          "waist",
                          figure.joint_density,
                          (figure.waist_height / 2.0) * sfac)

        frame.solids.append(solid)

        figure.waist = solid

        waist = solid.body
        waist.setPosition((x, y, z))

        joint = attachFixedJoint(frame, 'C', pelvis, waist)
        figure.pelvis.joint = joint

        # Torso

        h_tot += h
        h = figure.torso_ydim * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz
        solid = solidBox(world, space,
                         "torso",
                         figure.torso_density,
                         figure.torso_xdim * sfac,
                         figure.torso_ydim * sfac,
                         figure.torso_zdim * sfac)

        frame.solids.append(solid)

        figure.torso = solid

        torso = solid.body
        torso.setPosition((x, y, z))

        # Attach torso to waist joint and motor

        specs = figure.JointSpecs['waist']

        if frame.balljoints:
            joint = attachBallJoint(frame, 'C', torso, waist, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, 'C', torso, waist, specs)
            motor = attachUniversalAMotor(frame, joint, specs)

        # Create right and left arms

        h_tot += h
        dx = figure.torso_xdim * sfac / 2.0 + \
             figure.shoulder_radius * sfac

        (rx, ry, rz, rshoulder) = self.createFigureArm(figure,
                                                       'R',
                                                       px - dx, h_tot, pz)

        (lx, ly, lz, lshoulder) = self.createFigureArm(figure,
                                                       'L',
                                                       px + dx, h_tot, pz)

        # Attach shoulder joints and motors

        specs = figure.JointSpecs['shoulder']

        if frame.balljoints:
            joint = attachBallJoint(frame, 'R', torso, rshoulder, specs)
            motor = attachBallAMotor(frame, joint, specs)
            joint = attachBallJoint(frame, 'L', torso, lshoulder, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, 'R', torso, rshoulder, specs)
            motor = attachUniversalAMotor(frame, joint, specs)
            joint = attachUniversalJoint(frame, 'L', torso, lshoulder, specs)
            motor = attachUniversalAMotor(frame, joint, specs)

        # Neck joint

        h = figure.neck_height * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz
        solid = solidCone(world, space,
                          "neck",
                          figure.joint_density,
                          figure.neck_radius * sfac,
                          (figure.neck_radius / 2.0) * sfac,
                          figure.neck_height * sfac)

        frame.solids.append(solid)

        figure.neck = solid

        neck = solid.body
        neck.setPosition((x, y, z))

        joint = attachFixedJoint(frame, 'C', torso, neck)

        # Head

        h_tot += h
        h = 2.0 * figure.head_radius * sfac
        x = px
        y = h_tot + figure.head_radius * sfac
        z = pz
        solid = solidBall(world, space,
                          "head",
                          figure.head_density,
                          figure.head_radius * sfac)

        frame.solids.append(solid)

        figure.head = solid

        head = solid.body
        head.setPosition((x, y, z))

        hx = x
        hy = y
        hz = z

        # Attach head to neck joint and motor

        specs = figure.JointSpecs['neck']

        if frame.balljoints:
            joint = attachBallJoint(frame, 'C', head, neck, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, 'C', head, neck, specs)
            motor = attachUniversalAMotor(frame, joint, specs)

        return (hx, hy, hz)

    def createFigureLeg(self, figure, side, px, py, pz):
        """
        Creates a leg for the figure's frame in its world space.
    
        The position coordinates (px,py,pz) are used for bottom of the
        foot directly below center of hip joint.

        @param figure: A figure for which to attach a frame's leg.
        @type  figure: viewODE Figure object
        @param side: 'R' for right, 'L' for left.
        @type  side: char
        @param px: World space position x coordinate.
        @type  px: float
        @param py: World space position y coordinate.
        @type  py: float
        @param pz: World space position z coordinate.
        @type  pz: float

        @return: (x,y,z,body) -- position and body of hip joint.
        @rtype: tuple
        """

        world = figure.world
        space = figure.space
        frame = figure.frame
        sfac = figure.sfac

        # Toes

        h_tot = py
        x = px
        y = h_tot + figure.toes_ydim * sfac / 2.0
        z = pz + figure.foot_zdim * sfac + \
            figure.ball_radius * 2.0 * sfac + \
            figure.toes_zdim * 0.5 * sfac - \
            figure.ankle_radius * sfac
        solid = solidBox(world, space,
                         solidLabel(side, "toes"),
                         figure.limb_density,
                         figure.toes_xdim * sfac,
                         figure.toes_ydim * sfac,
                         figure.toes_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_toes = solid
        else:
            figure.l_toes = solid

        toes = solid.body
        toes.setPosition((x, y, z))

        # Ball of foot joint

        x = px
        y = h_tot + figure.ball_radius * sfac
        z = pz + figure.foot_zdim * sfac + \
            figure.ball_radius * sfac - \
            figure.ankle_radius * sfac
        solid = solidBall(world, space,
                          solidLabel(side, "ball"),
                          figure.joint_density,
                          figure.ball_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_ball = solid
        else:
            figure.l_ball = solid

        ball = solid.body
        ball.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, toes, ball)

        # Foot

        h = figure.foot_ydim * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz + figure.foot_zdim * sfac / 2.0 - \
            figure.ankle_radius * sfac
        solid = solidBox(world, space,
                         solidLabel(side, "foot"),
                         figure.limb_density,
                         figure.foot_xdim * sfac,
                         figure.foot_ydim * sfac,
                         figure.foot_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_foot = solid
        else:
            figure.l_foot = solid

        foot = solid.body
        foot.setPosition((x, y, z))

        # Attach foot to ball joint and motor

        specs = figure.JointSpecs['ball']

        if frame.balljoints:
            joint = attachBallJoint(frame, side, foot, ball, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, side, foot, ball, specs)
            motor = attachUniversalAMotor(frame, joint, specs)
            # joint = attachHingeJoint(frame,side,foot,ball,specs)
            # motor = attachHingeAMotor(frame,joint,specs)

        # Ankle joint

        h_tot += h
        h = 2.0 * figure.ankle_radius * sfac
        x = px
        y = h_tot + figure.ankle_radius * sfac
        z = pz
        solid = solidBall(world, space,
                          solidLabel(side, "ankle"),
                          figure.joint_density,
                          figure.ankle_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_ankle = solid
        else:
            figure.l_ankle = solid

        ankle = solid.body
        ankle.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, foot, ankle)

        # Shin

        h_tot += h
        h = figure.shin_ydim * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz
        if frame.slimbones:
            solid = solidRod(world, space,
                             solidLabel(side, "shin"),
                             figure.limb_density,
                             figure.shin_radius * sfac,
                             figure.shin_length * sfac)
        else:
            solid = solidBox(world, space,
                             solidLabel(side, "shin"),
                             figure.limb_density,
                             figure.shin_xdim * sfac,
                             figure.shin_ydim * sfac,
                             figure.shin_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_shin = solid
        else:
            figure.l_shin = solid

        shin = solid.body
        shin.setPosition((x, y, z))

        # Attach shin to ankle joint and motor

        specs = figure.JointSpecs['ankle']

        if frame.balljoints:
            joint = attachBallJoint(frame, side, shin, ankle, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, side, shin, ankle, specs)
            motor = attachUniversalAMotor(frame, joint, specs)
            # joint = attachHingeJoint(frame,side,shin,ankle,specs)
            # motor = attachHingeAMotor(frame,joint,specs)

        # Knee joint

        h_tot += h
        h = 2.0 * figure.knee_radius * sfac
        x = px
        y = h_tot + figure.knee_radius * sfac
        z = pz
        solid = solidBall(world, space,
                          solidLabel(side, "knee"),
                          figure.joint_density,
                          figure.knee_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_knee = solid
        else:
            figure.l_knee = solid

        knee = solid.body
        knee.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, shin, knee)

        # Thigh

        h_tot += h
        h = figure.thigh_ydim * sfac
        x = px
        y = h_tot + h / 2.0
        z = pz
        if frame.slimbones:
            solid = solidRod(world, space,
                             solidLabel(side, "thigh"),
                             figure.limb_density,
                             figure.thigh_radius * sfac,
                             figure.thigh_length * sfac)
        else:
            solid = solidBox(world, space,
                             solidLabel(side, "thigh"),
                             figure.limb_density,
                             figure.thigh_xdim * sfac,
                             figure.thigh_ydim * sfac,
                             figure.thigh_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_thigh = solid
        else:
            figure.l_thigh = solid

        thigh = solid.body
        thigh.setPosition((x, y, z))

        # Attach thigh to knee joint and motor

        specs = figure.JointSpecs['knee']

        if frame.balljoints:
            joint = attachBallJoint(frame, side, thigh, knee, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachHingeJoint(frame, side, thigh, knee, specs)
            motor = attachHingeAMotor(frame, joint, specs)

        # Hip joint

        h_tot += h
        x = px
        y = h_tot + figure.hip_radius * sfac
        z = pz
        solid = solidBall(world, space,
                          solidLabel(side, "hip"),
                          figure.joint_density,
                          figure.hip_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_hip = solid
        else:
            figure.l_hip = solid

        hip = solid.body
        hip.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, thigh, hip)

        return (x, y, z, hip)

    def createFigureArm(self, figure, side, px, py, pz):
        """
        Creates an arm for the figure's frame in its world space.
    
        The position coordinates (px,py,pz) are used for top of the
        shoulder directly above center of shoulder joint.

        @param figure: A figure for which to attach a frame's arm.
        @type  figure: viewODE Figure object
        @param side: 'R' for right, 'L' for left.
        @type  side: char
        @param px: World space position x coordinate.
        @type  px: number
        @param py: World space position y coordinate.
        @type  py: number
        @param pz: World space position z coordinate.
        @type  pz: number

        @return: (x,y,z,body) -- position and body of shoulder joint.
        @rtype: tuple
        """
        world = figure.world
        space = figure.space
        frame = figure.frame
        sfac = figure.sfac

        # Shoulder joint

        h_tot = py
        h = 2.0 * figure.shoulder_radius * sfac
        x = px
        y = h_tot - figure.shoulder_radius * sfac
        z = pz
        solid = solidBall(world, space,
                          solidLabel(side, "shoulder"),
                          figure.joint_density,
                          figure.shoulder_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_shoulder = solid
        else:
            figure.l_shoulder = solid

        shoulder = solid.body
        shoulder.setPosition((x, y, z))

        sx = x
        sy = y
        sz = z

        # Upper_arm

        h_tot -= h
        h = figure.upper_arm_ydim * sfac
        x = px
        y = h_tot - h / 2.0
        z = pz
        if frame.slimbones:
            solid = solidRod(world, space,
                             solidLabel(side, "upper_arm"),
                             figure.limb_density,
                             figure.upper_arm_radius * sfac,
                             figure.upper_arm_length * sfac)
        else:
            solid = solidBox(world, space,
                             solidLabel(side, "upper_arm"),
                             figure.limb_density,
                             figure.upper_arm_xdim * sfac,
                             figure.upper_arm_ydim * sfac,
                             figure.upper_arm_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_upper_arm = solid
        else:
            figure.l_upper_arm = solid

        upper_arm = solid.body
        upper_arm.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, upper_arm, shoulder)

        # Elbow joint

        h_tot -= h
        h = 2.0 * figure.elbow_radius * sfac
        x = px
        y = h_tot - h / 2.0
        z = pz
        solid = solidBall(world, space,
                          solidLabel(side, "elbow"),
                          figure.joint_density,
                          figure.elbow_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_elbow = solid
        else:
            figure.l_elbow = solid

        elbow = solid.body
        elbow.setPosition((x, y, z))

        # Attach upper_arm to elbow joint and motor

        specs = figure.JointSpecs['elbow']

        if frame.balljoints:
            joint = attachBallJoint(frame, side, upper_arm, elbow, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachUniversalJoint(frame, side, upper_arm, elbow, specs)
            motor = attachUniversalAMotor(frame, joint, specs)

        # Fore_arm

        h_tot -= h
        h = figure.fore_arm_ydim * sfac
        x = px
        y = h_tot - h / 2.0
        z = pz
        if frame.slimbones:
            solid = solidRod(world, space,
                             solidLabel(side, "fore_arm"),
                             figure.limb_density,
                             figure.fore_arm_radius * sfac,
                             figure.fore_arm_length * sfac)
        else:
            solid = solidBox(world, space,
                             solidLabel(side, "fore_arm"),
                             figure.limb_density,
                             figure.fore_arm_xdim * sfac,
                             figure.fore_arm_ydim * sfac,
                             figure.fore_arm_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_fore_arm = solid
        else:
            figure.l_fore_arm = solid

        fore_arm = solid.body
        fore_arm.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, fore_arm, elbow)

        # Wrist joint

        h_tot -= h
        h = 2.0 * figure.wrist_radius * sfac
        x = px
        y = h_tot - h / 2.0
        z = pz
        solid = solidBall(world, space,
                          solidLabel(side, "wrist"),
                          figure.joint_density,
                          figure.wrist_radius * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_wrist = solid
        else:
            figure.l_wrist = solid

        wrist = solid.body
        wrist.setPosition((x, y, z))

        # Attach fore_arm to wrist joint and motor

        specs = figure.JointSpecs['wrist']

        if frame.balljoints:
            joint = attachBallJoint(frame, side, fore_arm, wrist, specs)
            motor = attachBallAMotor(frame, joint, specs)
        else:
            joint = attachHingeJoint(frame, side, fore_arm, wrist, specs)
            motor = attachHingeAMotor(frame, joint, specs)

        # Hand

        h_tot -= h
        h = figure.hand_ydim * sfac
        x = px
        y = h_tot - h / 2.0
        z = pz
        solid = solidBox(world, space,
                         solidLabel(side, "hand"),
                         figure.limb_density,
                         figure.hand_xdim * sfac,
                         figure.hand_ydim * sfac,
                         figure.hand_zdim * sfac)

        frame.solids.append(solid)

        if side == 'R':
            figure.r_hand = solid
        else:
            figure.l_hand = solid

        hand = solid.body
        hand.setPosition((x, y, z))

        joint = attachFixedJoint(frame, side, hand, wrist)

        return (sx, sy, sz, shoulder)

    def updateFrameSolidsPosVelAcc(self, t, tstep):
        """
        Update frame solids position data points and use to estimate updated
        velocity and acceleration.
        :param t: simulation time
        :param tstep: simulation time step
        :return: none
        """
        KIDX = [(0, 1, 2), (1, 2, 0), (2, 0, 1)]  # cmpos indexes for update count
        for s in self.solids:
            cmpos = s.body.getPosition()
            if s.knt == 0:
                # initialize
                s.t0 = t
                s.cmpos[0] = cmpos
                cmvel = (0.0, 0.0, 0.0)
                cmacc = (0.0, 0.0, 0.0)
            elif s.knt == 1:
                # use 2-point difference expressions
                s.cmpos[1] = cmpos
                cmvel = vecDivS(vecSub(s.cmpos[1], s.cmpos[0]), tstep)
                cmacc = vecDivS(vecSub(cmvel, s.cmvel), tstep)
            else:
                # use 3-point central difference method
                k = (s.knt + 1) % 3
                idx = KIDX[k][0:]
                s.cmpos[idx[2]] = cmpos
                cmvel = vecDivS(vecSub(s.cmpos[idx[2]], s.cmpos[idx[0]]), 2 * tstep)
                cmacc = vecDivS(vecSub(vecAdd(s.cmpos[idx[2]], s.cmpos[idx[0]]), vecMulS(s.cmpos[idx[1]], 2)),
                                tstep * tstep)
                if self.figure.actions.debug and s.label == "pelvis":
                    print("%s: knt = %4d  k = %1d  idx = %1d, %1d, %1d" %
                          (s.label, s.knt, k, idx[0], idx[1], idx[2]))
                    print("s.cmpos[idx[0]] = %12.9f, %12.9f, %12.9f" %
                          (s.cmpos[idx[0]][0], s.cmpos[idx[0]][1], s.cmpos[idx[0]][2]))
                    print("s.cmpos[idx[1]] = %12.9f, %12.9f, %12.9f" %
                          (s.cmpos[idx[1]][0], s.cmpos[idx[1]][1], s.cmpos[idx[1]][2]))
                    print("s.cmpos[idx[2]] = %12.9f, %12.9f, %12.9f" %
                          (s.cmpos[idx[2]][0], s.cmpos[idx[2]][1], s.cmpos[idx[2]][2]))
                    print("s.cmvel = %9.5f, %9.5f, %9.5f" %
                          (cmvel[0], cmvel[1], cmvel[2]))
                    print("s.cmacc = %9.5f, %9.5f, %9.5f" %
                          (cmacc[0], cmacc[1], cmacc[2]))
            s.knt += 1
            s.cmvel = cmvel
            s.cmacc = cmacc
