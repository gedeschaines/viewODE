## README ##

### viewODE ###
 
A Python program to view and control a simulated humanoid robotic figure using PyODE for modeling articulated rigid body dynamics, PyOpenGL for 3D rendering and Pillow (PIL) for image capture and conversion. The program was developed as a simplistic virtual dynamics environment to explore the capabilities of the Open Dynamics Engine (ODE) library in modeling a rudimentary humanoid robot and is not intended to be a robust robot simulation, development and experimentation tool such as [V-REP](http://www.coppeliarobotics.com/index.html), [GAZEBO](http://gazebosim.org/), [ROBOPY](https://github.com/adityadua24/robopy) or [pybotics](https://github.com/nnadeau/pybotics).

### Repository Directory Structure ###

The main program file is **viewODE.py**. Other Python files are object class modules (Actions, Capture, Control, Figure, Frame, Render, Select and Solids), static procedure modules (Joints, Motors and vecMath), and various utility programs and scripts.

* The root directory contains the main program file and imported object class and static procedure module files.
* The ./docs subdirectory contains a keyboard and mouse input options guide, design notes and howto tutorials.
* The ./test subdirectory contains unit test driver programs and scripts.
* The ./util subdirectory contains Python and shell scripts for converting captured JPEG image files to animated GIFs and MP4 videos, and to generate **xgraph** data files from debug print text files.

### Execution Prerequisites ###

The **viewODE** program was originally developed with Python 2.5.2, ODE 0.7 and PyODE 1.2.0 on a Windows XP platform, but recent development efforts have been with Python 2.7.6 and 3.4.3 on a Ubuntu 14.04 platform.

* Python 2.7.6 or 3.4.3
* ODE 0.11.1 and PyODE 1.2.1 for Python 2.7.6
* ODE 0.12 with Python binding for Python 2.7.6 or 3.4.3
* PyOpenGL 3.0.2
* Pillow 2.3.0
* Mouse with 3 buttons (left for picking, middle for view rotation, right for start/pause)

### Execution Overview ###

The **viewODE** program presents an interactive display for three dimensional (3D) rendering of a dynamically active humanoid robotic figure comprised of rigid body solids connected with motor actuated joints. The view point and robotic figure can be manipulated using the keyboard and mouse.

* Entering 'python viewODE.py' on a command shell line when in the root directory will open a **viewODE** window in which will be displayed a ground plane grid with the world reference coordinate system frame +X, +Y and +Z axes positioned at the world space origin as shown [here](./docs/start_image.png). Program status messages and debug output are printed to the command shell unless otherwise redirected.
* Pressing the 'Z' key or clicking the right mouse button will initiate the simulation by displaying a humanoid figure standing on the ground plane and a solid sphere (target) positioned in front of the figure as shown [here](./docs/begin_image.png). Mouse motion while the middle button is pressed will rotate the view about the world space X and Y axes. Apparent rotation of the ground plane does not affect dynamics since gravitational force always remains in the -Y direction.
* The figure is initially in a standing state and will actively attempt to remain standing by adjusting torques on the ankle, knee, hip, waist and neck joint motors. While the simulation is running, the user can change the following with key presses as described in the **./docs/InputOptions.txt** file.
    * arm and leg bone shape (box or rod)
    * joint type (hinge or ball)
    * action state (standing, suspended, striding, reaching, kicking or walking)
    * joint damping mode (none, rotational or feedback torque)
    * manipulated joint forcing torque modes (non-forced axis limiting and maintain joint rotation)
    * active motor axis (none or +/- x, y or z)
    * rendering mode (orthographic or perspective, selection volume rendering)
    * simulation modes (paused, reset, image capture)
    * debug output (render, actions and control)
* Pressing the left mouse button while the mouse cursor is on a solid will select the solid for manipulation. The selected solid is rendered as a wire frame. If the target sphere is selected, it can be moved to apply an impact force on the figure or to influence the figure's reaction while in the reaching or kicking mode. If the selected solid is attached to the figure's frame, then the figure is placed in the suspension state and a force or torque is applied to the solid.
    * A grabbing force is applied to solids corresponding to these body parts: toes, foot, shin, thigh, pelvis, torso, hand, forearm, upper arm and head.
    * A motor torque is applied to joints corresponding these body parts: ball of foot, ankle, knee, hip, waist, wrist, elbow, shoulder and neck.
* To collect debug print data, redirect shell output to a text file when invoking viewODE.py such as: 'python viewODE.py 1>output.txt 2>&1'. Specific joint data records can be extracted from the output file and written to another text file using a utility such as **grep**, and the extracted joint data records file can then be processed with **./util/txt2dat.py** to produce a data file suitable for input to the **xgraph** program.
* The simulation can be paused/resumed by pressing the 'Z' key or by pressing/holding/releasing the right mouse button.
* To terminate the program, press the 'esc' key or close the **viewODE** window.

### Attributions ###

* Matthias Baas and Pierre Gay for the [tutorial3.py](https://sourceforge.net/projects/pyode/) program which was used as a basis for the simulation loop, solid rendering, and collision, keypress, mouse and idle callbacks.
* Matt Heinzen for the [ragdoll-pyode-tutorial.py](http://monsterden.net/software/ragdoll-pyode-tutorial) program which was used as a basis for vector math functions and jointed rigid body modeling of a humanoid figure.

### Support ###

* For install or usage issues concerning ODE and PyODE, consult the appropriate document, wiki, forum or mailing list provided at the SourceForge projects for [Open Dynamics Engine](https://sourceforge.net/projects/opende/) and [Python-ODE Bindings](https://sourceforge.net/projects/pyode/) or at the original [ODE](http://ode.org/) and [PyODE](http://pyode.sourceforge.net/) web sites. Providers of distribution packages for ODE and PyODE may also be helpful in resolving installation issues.
* For guidance on exploring the features and capabilites of **viewODE**, consult the howto tutorials provided in the **./docs** subdirectory.