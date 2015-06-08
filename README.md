MavConnection
=============

A library to connect to APM through MAVLink protocol

To make the library run
make lib

To make the test run
make test

To run the test create a simbolic to the ArduCopter.elf and to the Simulator which can be found in
https://github.com/ptsneves/QuadSim_OpenGL. Consider $ARDUPILOT_REPOSITORY and $QUADSIM_OPENGL_REPOSITORY to be the
paths to repositories.
If the links are wrong the test will silently fail. Test manually the link by running the simulator and the 
ArduCopter.elf. I know it is bad but you can correct and request a push to ProcessSpawner.cpp :D

Warning: If you create symbolic links like below with paths in relative format you must add the -r option

mkdir Simulator
ln -s $ARDUPILOT_REPOSITORY/ ./Simulator/ardupilot
ln -s $QUADSIM_OPENGL_REPOSITORY/Simulator.py Simulator.py
