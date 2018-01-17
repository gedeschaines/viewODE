## README ##

### viewODE ###
 
A Python program to view and control a simulated humanoid robotic figure using PyODE for modeling articulated rigid body dynamics, PyOpenGL for 3D rendering and Pillow (PIL) for image capture and conversion. The program was developed as a simplistic virtual dynamics environment to explore the capabilities of the Open Dynamics Engine (ODE) library in modeling a rudimentary humanoid robot. 

### Repository Directory Structure ###

The main program file is **viewODE.py**. Other Python files are object class modules (Actions, Capture, Control, Figure, Frame, Render, Select and Solids), static procedure modules (Joints, Motors and vecMath), and various utility programs and scripts.

* The root directory contains the main program file and imported object class and static procedure module files.
* The ./docs subdirectory contains keyboard and mouse input options guide, design notes and howto tutorials.
* The ./test subdirectory contains unit test driver programs and scripts.
* The ./util subdirectory contains Python and shell scripts for converting captured JPEG image files to animated GIFs and MP4 videos, and to generate **xgraph** data files from debug print text files.

### Execution Prerequisites ###

The **viewODE** program was originally developed with Python 2.5, ODE 0.7 and PyODE 1.2.0 on a Windows XP platform, but recent development efforts have been with Python 2.7 and 3.4 on a Ubuntu 14.04 platform.

* Python 2.7.6+ or 3.4.3+
* ODE 0.11.1 and PyODE 1.2.1 for Python 2.7.6+
* ODE 0.12+ with Python binding for Python 2.7.6+ or 3.4.3+
* PyOpenGL 3.0.2
* Pillow 2.3.0
* Mouse with 3 buttons (left for picking, middle for view rotation, right for start/pause)

### Execution Instructions ###

The **viewODE** program presents an interactive display for three dimensional (3D) rendering of a dynamically active humanoid robotic figure comprised of rigid body solids connected with motor actuated joints. The view point and robotic figure can be manipulated using the keyboard and mouse.

* Entering 'python viewODE.py' on a command shell line when in the root directory will open a **viewODE** window in which will be displayed a ground plane grid with the world reference system coordinate frame +X, +Y and +Z axes positioned at the world space origin. Program status messages and debug output are printed to the command shell unless otherwise redirected.
* Pressing the 'Z' key or clicking the right mouse button will initiate the simulation by displaying a humanoid figure standing on the ground plane and solid sphere positioned in front of the figure as shown [here](./docs/start_image.html). Mouse motion while the middle button is pressed will rotate the view about the world space X and Y axes. Apparent rotation of the ground plane does not affect dynamics since gravitational force always remains in the -Y direction.
* The figure is initially in a standing state and will actively attempt to remain standing by adjusting torques on the ankle, knee, hip, waist and neck joints. While the simulation is running, the user can change the figure arm and leg bone shape (box or rod), joint type (hinge or ball), action state (standing, suspended, stride testing, reaching, kicking or walking), joint damping mode (none, rotational or feedback torque), active motor axis (none or +/- x, y or z), rendering projection mode (orthographic or perspective), simulation modes (paused, reset, image capture, debug print, etc.) with key presses as described in the **./docs/InputOptions.txt** file. Left mouse button presses with the cursor placed on a figure body solid will place the figure in suspended mode and apply a torque to a joint if the selected solid is a sphere or cone attached to the figure's frame, otherwise a force is applied to the selected solid.
* To collect debug print data, redirect shell output to a text file when invoking viewODE.py such as: 'python viewODE.py 1>output.txt 2>&1'. Specific joint data records can be extracted from the output file and written to another text file using a utility such as **grep**, and the extracted joint data records file can then be processed with **./util/txt2dat.py** to produce a data file suitable for input to the **xgraph** program.
* The simulation can be paused/resumed by pressing the 'Z' key or pressing/holding/releasing the right mouse button.
* To terminate the program, press the 'esc' key or close the **viewODE** window.

### Attributions ###

* Matthias Baas and Pierre Gay for the [tutorial3.py](https://sourceforge.net/projects/pyode/) program which was used as a basis for the simulation loop, solid rendering, and collision, keypress, mouse and idle callbacks.
* Matt Heinzen for the [ragdoll-pyode-tutorial.py](http://monsterden.net/software/ragdoll-pyode-tutorial) program which was used as a basis for vector math functions and jointed rigid body modeling of a humanoid figure.

### Support ###

* For install or usage issues concerning ODE and PyODE, consult the appropriate document, wiki, forum or mailing list provided at the SourceForge projects for [Open Dynamics Engine](https://sourceforge.net/projects/opende/) and [Python-ODE Bindings](https://sourceforge.net/projects/pyode/) or at the original [ODE](http://ode.org/) and [PyODE](http://pyode.sourceforge.net/) web sites. Providers of distribution packages for ODE and PyODE may also be helpful in resolving installation issues.
* For guidance on exploring the features and capabilites of **viewODE**, consult the howto tutorials provided in the **./docs** subdirectory.