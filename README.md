## README ##

### viewODE ###
 
Python program to view and control a simulated humanoid robotic figure using PyODE for modeling jointed rigid body dynamics, PyOpenGL for 3D rendering and Pillow (PIL) for image capture and conversion. The program was originally developed as a simplistic virtual dynamics environment to explore the capabilities of ODE in modeling a humanoid robot. 

### Repository Directory Structure ###

The main program file is viewODE.py. Other python files are object class modules (Actions, Capture, Control, Figure, Frame, Render, Select, Solids), static procedure modules (Joints, Motors and vecMath) or utility programs and scripts.

* The root directory contains the main program file and associated object class and static procedure module files.
* The ./docs subdirectory contains program usage instructions and design documentation.
* The ./test subdirectory contains unit test driver programs and scripts.
* The ./util subdirectory contains scripts for converting captured JPEG image files to animated GIFs and MP4s, and to generate **xgraph** data files from debug print text files.

### Execution Prerequisites ###

* libode-0.11.1
* Python 2.7.6
* PyODE 1.2.1
* PyOpenGL 3.0.2
* Pillow 2.3.0
* Mouse with 3 buttons (left for picking, middle for view rotation, right for start/pause)

### Execution Instructions ###

* Entering 'python2.7 viewODE.py' on a command shell line when in the root directory will open a viewODE window in which will be displayed a ground plane grid with the world reference system coordinate frame +X, +Y and +Z axes positioned at the world space origin. Program status messages and debug output are printed to the command shell unless redirected.
* Pressing the 'Z' key or clicking the right mouse button will initiate the simulation by displaying a humanoid figure standing on the ground plane and solid sphere positioned in front of the figure. Mouse motion while the middle button is pressed will rotate the view about the world space X and Y axes. Apparent rotation of the ground plane does not affect dynamics since gravitational force always remains in the -Y direction.
* The figure is initially in a standing state and will actively attempt to remain standing by adjusting torques on the ankle, knee, hip, waist and neck joints. While the simulation is running, the user can change the figure arm and leg bone shape (box or rod), joint type (hinge or ball), action state (standing, suspended, stride testing, reaching, kicking or walking), joint damping mode (none, rotational or feedback torque), active motor axis (none or +/- x, y or z), rendering projection mode (orthographic or perspective), simulation modes (paused, reset, image capture, debug print, etc.) with key presses as described in the ./docs/InputOptions.txt file. Left mouse button presses with the cursor placed on a figure body solid will place the figure in suspended mode and apply a torque to a joint if the selected solid is a sphere or cone attached to the figure's frame, otherwise a force is applied to the selected solid.
* To collect debug print data, redirect shell output to a text file when invoking viewODE.py such as: 'python2.7 viewODE.py 1>output.txt 2>&1'. Specific joint data records can be extracted from the output file and written to another text file using a utility such as **grep**, and the extracted joint data records file can then be processed with ./util/txt2dat.py to produce a data file suitable for input to the **xgraph** program.

### Attributions ###

* Matthias Baas and Pierre Gay for the [tutorial3.py](https://sourceforge.net/projects/pyode/) program which was used as a basis for the simulation loop, solid rendering, and collision, keypress, mouse and idle callbacks in viewODE.
* Matt Heinzen for the [ragdoll-pyode-tutorial.py](http://monsterden.net/software/ragdoll-pyode-tutorial) program which was used as a basis for vector math functions and jointed rigid body modeling of a humanoid figure.

