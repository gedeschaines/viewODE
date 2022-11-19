#!/usr/bin/env python3

# viewODE.py
#
# View and manipulate a humanoid robotic figure simulated using PyODE for
# modeling jointed rigid body dynamics, PyOpenGL for 3D rendering, and PIL
# for image capture.
#
# Originally by Gary Deschaines, 2009.
#
# Attributions
#
# + Matthias Baas and Pierre Gay for the Python-ODE Bindings examples program
#   tutorial3.py available at https://sourceforge.net/projects/pyode/ which was
#   used as a basis for the main simulation loop, solid rendering, and collision,
#   keypress, mouse and idle callbacks.
# + Matt Heinzen for PyODE Ragdoll Physics Tutorial program ragdoll-pyode-tutorial.py
#   available at http://monsterden.net/software/ragdoll-pyode-tutorial which was used
#   as a basis for vector math functions and jointed rigid body modeling of a humanoid
#   figure.
#
# Disclaimers
#
#   See the file DISCLAIMER-GaryDeschaines
#   See the file DISCLAIMER-PyODE
#   See the file DISCLAIMER-MattHeinzen

from __future__ import print_function
from sys import argv, exit, modules, path, stderr, stdout
from os import getcwd

import time

#
# Import OpenGL modules for rendering and ODE module for dynamics.

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *
except ModuleNotFoundError or ImportError as ee:
    print("Error: PyOpenGL not installed properly !! - {0}".format(ee.msg), file=stderr)
    exit(1)

try:
    import ode
except ModuleNotFoundError or ImportError as ee:
    print("Error: PyODE not installed properly !! - {0}".format(ee.msg), file=stderr)
    exit(1)

#
# Import viewODE modules for figure, solids, rendering and vector math.

path.insert(1, getcwd())

try:
    from Figure import *
    from Solids import *
    from Render import *
    from vecMath import *
except ModuleNotFoundError or ImportError as ee:
    print("Error: {0}".format(ee.msg), file=stderr)

if not (modules["Figure"] and modules["Solids"] and modules["Render"] and modules["vecMath"]):
    print("Error: viewODE not installed properly !!", file=stderr)
    exit(1)

#
# Static procedures to support main simulation loop processes


def create_target_object(world, space, px, py, pz):
    """
    Create a target object at the specified position (px,py,pz) in the
    given ODE world space.

    Returns (x,y,z),tgt_obj

    @param world: An ODE world returned by call to ode.World().
    @type  world: ODE World object
    @param space: An ODE space returned by call to ode.Space().
    @type  space: ODE Space object
    @param px: World space position x coordinate.
    @type  px: float
    @param py: World space position y coordinate.
    @type  py: float
    @param pz: World space position z coordinate.
    @type  pz: float

    @return: (x,y,z),tgt_obj -- position coordinates in world space for
             center of the target body and the target's ODE body object.
    @rtype:  tuple
    """
    r = 0.25
    x = px
    y = py
    z = pz
    density = 100
    solid = solidBall(world, space, "target", density, r)
    tgt_obj = solid.body
    tgt_obj.setPosition((x, y, z))

    return (x, y, z), tgt_obj


def delete_target_object(tgt_obj):
    """
    Delete target object created by the L{create_target_object} method.

    @param tgt_obj: The ODE body object created by call to create_target_object.
    @type  tgt_obj: ODE Body object

    @return: empty target object
    @rtype: NoneType
    """
    if tgt_obj:
        tgt_obj.disable()
        geom = tgt_obj.solid.geom
        geom_space = geom.getSpace()
        geom_space.remove(geom)
        tgt_obj = None
    return tgt_obj


def resetSim(mode):
    """
    Perform operations associated with given reset mode enumeration

    @param mode: Enumeration of a reset mode.
    @type mode: int
    """
    global RESET_MODE, PAUSE
    global ode_contactgroup
    global state
    global ode_world
    global renderer
    global figure
    global target

    # Reset simulation state
    state = 0
    # PAUSE = True

    if mode == RESET_MODE['hard']:
        # Reset world gravity vector
        ode_world.setGravity((0.0, -9.81, 0.0))
        # Reset figure configuration
        print(" ")
        print("**** Figure Configuration Reset ****")
        figure.resetConfig()
        # Reset rendering and capturing
        print(" ")
        print("**** Rendering and Capturing Reset ****")
        renderer.resetConfig()

    # Reset figure animation state
    if mode != RESET_MODE['exit']:
        if figure.getAnimationState() != figure.INITIALIZE:
            print(" ")
            print("**** Figure Animation Reset ****")
            figure.resetAnimation()

    # Empty contactgroup joints
    ode_contactgroup.empty()

    # Delete figure frame elements
    figure.delete()

    # Delete target
    target = delete_target_object(target)

    if mode != RESET_MODE['exit']:
        print(" ")
        print("**** Simulation Reset ****")
        printConfig()


def null_display():
    return


def exitSim(reset_mode):
    """
    Performs reset operations prior to simulation exit.

    @param reset_mode: Enumeration of a reset mode.
    @type  reset_mode: int
    """
    resetSim(reset_mode)
    ode.CloseODE()
    stdout.flush()
    stderr.flush()
    glutDisplayFunc(null_display)
    glutIdleFunc(None)
    if renderer.windowID:
        glutDestroyWindow(renderer.windowID)
    if callable(glutLeaveMainLoop):
        glutLeaveMainLoop()


def printConfig():
    """
    Print renderer and figure configurations.
    """
    global renderer
    global figure

    print("---------- Rendering Configuration ----------")
    renderer.printConfig()
    print("---- Figure Configuration ----")
    figure.printConfig()
    print("---------------------------------------------")


def _keyfunc(key, x, y):
    """
    Keypress handler.
  
    This method is passed to the Render class constructor in order to
    be registered as the glutKeyboardFunc callback.

    @param key: Keypress code.
    @type  key: char
    @param x: Window x coordinate.
    @type  x: int
    @param y: Window y coordinate.
    @type  y: int
    """
    global RESET_MODE, GRAV_FIXED, PAUSE
    global renderer
    global figure
    global state
    global ode_world

    key = key.decode()

    if key == '\x1b':
        exitSim(RESET_MODE['exit'])
        return

    if figure.doKeyPress(key):
        if figure.getAnimationState() == 0:
            resetSim(RESET_MODE['soft'])
        return

    if renderer.doKeyPress(key):
        return

    if key == 'a' or key == 'A':
        # Assemble object
        resetSim(RESET_MODE['soft'])
    elif key == 'g' or key == 'G':
        # Toggle Render debug print
        renderer.toggleDebug()
    elif key == 'h' or key == 'H':
        # Toggle Actions debug print
        figure.toggleActionsDebug()
    elif key == 'j' or key == 'J':
        # Toggle Control debug print
        figure.toggleControlDebug()
    elif key == 'x' or key == 'X':
        # Reset simulation
        resetSim(RESET_MODE['hard'])
    elif key == 'z' or key == 'Z':
        # Toggle simulation pause state
        PAUSE = not PAUSE
        if PAUSE:
            print("*** Simulation pause")
        else:
            print("*** Simulation resume")
    elif key == '/':
        # Toggle fixed gravity
        GRAV_FIXED = not GRAV_FIXED
        if GRAV_FIXED:
            print("*** Fixed gravity on")
            ode_world.setGravity((0.0, -9.81, 0.0))
        else:
            print("*** Fixed gravity off")


def _mousefunc(b, s, mx, my):
    """
    Mouse button handler.
  
    This method is passed to the Render class constructor in order to
    be registered as the glutMouseFunc callback.

    @param b: Mouse button code.
    @type  b: int
    @param s: Mouse button state.
    @type  s: int
    @param mx: Mouse cursor position x coordinate.
    @type  mx: int
    @param my: Mouse cursor position y coordinate.
    @type  my: int
    """
    global renderer
    global PAUSE

    if renderer.doMouseButton(b, s, mx, my):
        # Left or right mouse button press/release
        return

    if b == GLUT_RIGHT_BUTTON and s == GLUT_DOWN:
        PAUSE = True
    elif b == GLUT_RIGHT_BUTTON and s == GLUT_UP:
        PAUSE = False


def grab(body, timestep):
    """
    Grabs a body solid.

    A lateral force is applied to the grabbed body solid in order to manipulate
    the body.

    @param body: The body returned by the Select pick processing.
    @type  body: ODE Body object
    @param timestep: The simulation time step.
    @type  timestep: float
    """
    global ode_world
    global renderer
    global figure
    global target

    if body:

        # Get picked body info
        (body, near, far) = renderer.selector.getPickedBodyInfo()

        # Calculate selection volume near and far vectors
        renderer.selector.calcSelectNvecFvecCvec()

        # Get previous selection volume near and far vectors
        (prev_n_vec, prev_f_vec) = renderer.selector.getPrevNvecFvec()

        if prev_n_vec:

            # Get current selection volume near and far vectors
            (n_vec, f_vec) = renderer.selector.getSelectNvecFvec()

            # Calculate grab direction vector
            d_vec = vecSub(n_vec, prev_n_vec)
            d_mag = vecMag(d_vec)
            if d_mag > 0.01:
                d_vec = vecMulS(d_vec, 1.0 / d_mag)
            else:
                d_mag = 0.0
                d_vec = (0.0, 0.0, 0.0)

            # Apply grab force
            if body == target:
                if d_mag > 0.0:
                    # grab force
                    f = body.mass * (1.0 / timestep)
                    f_vec = vecMulS(d_vec, f)
                    body.addForce(f_vec)
            elif not figure.frame.joints:
                # Collapsed figure
                if d_mag > 0.0:
                    # grab force
                    f = body.mass * (1.0 / timestep)
                    f_vec = vecMulS(d_vec, f)
                    body.addForce(f_vec)
                else:
                    # levitation force
                    (gx, gy, gz) = ode_world.getGravity()
                    body.addForce((gx, -body.mass * gy, gz))
            else:
                # Connected figure -- do not apply force to joint
                if not body.solid.joint:
                    if d_mag > 0.0:
                        # grab force
                        f = figure.getTotMass() * (0.2 / timestep)
                        f_vec = vecMulS(d_vec, f)
                        body.addForce(f_vec)


def near_callback(args, geom1, geom2):
    """
    Callback function for collide() method.

    This function checks if the given geoms do collide and creates contact
    joints if they do.

    @param args: ODE world and contactgroup
    @type  args: tuple
    @param geom1: geometry of 1st body
    @type  geom1: ODE Geometry object
    @param geom2: geometry of 2nd body
    @type  geom2: ODE Geometry objec
    """
    global figure

    body1, body2 = geom1.getBody(), geom2.getBody()
    if body1 is None:
        body1 = ode.environment
    if body2 is None:
        body2 = ode.environment
    if body1 == body2:
        return
    if ode.areConnected(body1, body2):
        return

    # Check if the objects do collide
    contacts = ode.collide(geom1, geom2)

    # Create contact joints
    world, contactgroup = args
    for c in contacts:
        (pos, nrm, depth, g1, g2) = c.getContactGeomParams()
        if isinstance(g2, ode.GeomPlane):
            # save foot contacts with ground for COP calculation.
            if g1.solid.label == "R_foot" or \
                    g1.solid.label == "R_ball" or \
                    g1.solid.label == "R_toes":
                c.setBounce(0.01)
                c.setMu(15000.0)
                figure.appendCGP('R', (pos, nrm))
            elif g1.solid.label == "L_foot" or \
                    g1.solid.label == "L_ball" or \
                    g1.solid.label == "L_toes":
                c.setBounce(0.01)
                c.setMu(15000.0)
                figure.appendCGP('L', (pos, nrm))
            else:
                c.setBounce(0.2)
                c.setMu(1500.0)
        else:
            flags = ode.ContactSoftCFM
            c.setMode(flags)
            c.setSoftCFM(0.2E-2)
            c.setBounce(0.2)
            c.setMu(1500.0)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(body1, body2)


def _idlefunc():
    """
    ODE simulation loop.
  
    This method is passed to the Render class constructor in order to be
    registered as the glutIdleFunc callback.
    """
    global PAUSE, GRAV_FIXED
    global renderer, frame_rate
    global ode_world, ode_space, ode_contactgroup
    global figure, target
    global state, lasttime
    global t, t_msec, dt, dt_msec, nstep, tstep, tstep_msec, tstop, tstop_msec

    t_wait = dt - (time.time() - lasttime)
    if t_wait > 0:
        time.sleep(t_wait)

    if not PAUSE:

        # State 0: Create ODE figure and target
        if state == 0:
            # Very small y offset added to mitigate dynamic perturbations due
            # to collision contact between bottom of feet and ground plane.
            figure.createFrame(0.0, 0.001, -2.0)
            (tgtpos, target) = create_target_object(ode_world,
                                                    ode_space,
                                                    0.0, 3.5, 2.0)
            target.setGravityMode(0)
            renderer.setRenderFigure(figure)
            renderer.setRenderTarget(target)
            if renderer.captor:
                # Zero capture millisecond counter
                renderer.captor.zeroMsecCounter()

            # Transition to animation state
            state = 1
            t = 0.0
            t_msec = 0.0

        # Simulate object dynamics
        for n in range(nstep):

            if state == 1:
                # Animation state -- check for picked body
                body = renderer.selector.getPickedBody()
                if body:
                    # Update grab forces
                    grab(body, tstep)
                # Update figure animation
                figure.updateAnimation(body, target, t, tstep)

            # Change gravity direction based on world floor plane rotation about X-axis
            if not GRAV_FIXED:
                gvec = (0.0, -9.81, 0.0)
                radx = -renderer.rotationX * RPD
                gvecx = (gvec[0], gvec[1] * cos(radx) - gvec[2] * sin(radx), gvec[1] * sin(radx) + gvec[2] * cos(radx))
                ode_world.setGravity(gvecx)

            # Detect collisions and create contact joints
            figure.createCGP('R')
            figure.createCGP('L')
            ode_space.collide((ode_world, ode_contactgroup), near_callback)
            figure.setCOP('R')
            figure.setCOP('L')

            if figure.actions.debug:
                cmpos = figure.getCenterOfMassPos()
                print("cmpos : t=%8.4f, x=%8.4f, y=%8.4f, z=%8.4f" % (t, cmpos[0], cmpos[1], cmpos[2]))
                cmvel = figure.getCenterOfMassVel()
                print("cmvel : t=%8.4f, x=%8.4f, y=%8.4f, z=%8.4f" % (t, cmvel[0], cmvel[1], cmvel[2]))
                cmacc = figure.getCenterOfMassAcc()
                print("cmacc : t=%8.4f, x=%8.4f, y=%8.4f, z=%8.4f" % (t, cmacc[0], cmacc[1], cmacc[2]))
                copR = figure.getCOP('R')
                if vecMag(copR) > 0.0:
                    copR = (copR[0] - cmpos[0], copR[1], copR[2] - cmpos[2])
                print("copR : t=%8.4f, x=%8.4f, y=%8.4f, z=%8.4f" % (t, copR[0], copR[1], copR[2]))
                copL = figure.getCOP('L')
                if vecMag(copL) > 0.0:
                    copL = (copL[0] - cmpos[0], copL[1], copL[2] - cmpos[2])
                print("copL : t=%8.4f, x=%8.4f, y=%8.4f, z=%8.4f" % (t, copL[0], copL[1], copL[2]))
                zmp = figure.getZMP()
                print("zmp : t=%8.4f, x=%8.4f, y=%8.4f, z=%8.4f" % (t, zmp[0], zmp[1], zmp[2]))
                figure.calcPelvisSumFandT()

            # Simulation step
            ode_world.step(tstep)

            # Remove all contact joints
            ode_contactgroup.empty()

            # Update simulation time
            t += tstep
            t_msec += tstep_msec

        if renderer.captor:
            # Update capture millisecond counter
            renderer.captor.updateMsecCounter(dt_msec)

        # Update display
        glutPostRedisplay()

        # Check simulation stop time
        if (tstop > 0.0) and (t_msec > tstop_msec):
            print("Simulation time %.3f exceeds stop time %.3f." % (t, tstop))
            exitSim(RESET_MODE['exit'])

    lasttime = time.time()

#
# Simulation main


if __name__ == '__main__':

    # Simulation reset mode:
    #   soft -- resets figure to current config/states
    #   hard -- resets figure, rendering and capturing
    #           to initial config/states
    #   exit -- exits simulation
    RESET_MODE = {'soft': 0, 'hard': 1, 'exit': 2}

    # Get simulation stop time from 1st command argument
    tstop = -1.0
    if len(argv) > 1:
        tstop = float(argv[1])
    tstop_msec = tstop * 1000.0

    # ODE simulation loop timing, counters and state
    PAUSE = True
    fps = 100
    t = 0.0
    t_msec = 0
    dt = 1.0 / float(fps)
    dt_msec = int(dt * 1000.0)
    nstep = 4
    tstep = dt / float(nstep)
    tstep_msec = tstep * 1000.0
    state = 0

    # Capture parameters
    frame_rate = 25  # Note: frame_rate should be a divisor of fps
    frame_time = int(1000.0 / float(frame_rate))        # in milliseconds rounded
    frame_time = int(dt_msec * (frame_time / dt_msec))  # to multiple of dt_msec
    filename = "img"

    # Initialize OpenGL renderer-selector and PIL captor
    screenx = 50
    screeny = 50
    width = 800
    height = 600
    renderer = Render(screenx, screeny,
                      width, height,
                      _idlefunc,
                      _keyfunc,
                      _mousefunc,
                      frame_time,
                      filename)

    # Create ODE world object
    GRAV_FIXED = True
    ode_world = ode.World()
    ode_world.setGravity((0.0, -9.81, 0.0))
    ode_world.setERP(0.2)
    ode_world.setCFM(0.2E-3)

    # Create ODE space object
    ode_space = ode.Space()

    # Create a plane geom which prevent the objects
    # from falling forever
    floor = ode.GeomPlane(ode_space, (0.0, 1.0, 0.0), 0)

    # A ODE joint group for the contact joints that are
    # generated whenever two bodies collide
    ode_contactgroup = ode.JointGroup()

    # Globals for ODE figure and target body information
    sfac = 1.0
    figure = Figure(ode_world, ode_space, floor, sfac)
    target = None

    # Print rendering, physical and dynamics configurations
    printConfig()

    if PAUSE:
        print("Press 'Z' key to begin simulation.")

    lasttime = time.time()

    # Begin simulation
    glutMainLoop()

    exit(0)
