
This is a simple command-line simulator that works with px4_sitl to 
simulate a flying vehicle. 

### Background

If you want to simulate flights with a PX4 (or ArduPilot) based controller, and test your PX4 firmware, you [have many options for simulation]( https://dev.px4.io/en/simulation/).  The [PX4/Firmware](https://github.com/PX4/Firmware)/Dronecode maintainers recommend AirSim, Gazebo, or jMAVSim.

Most of the recommended simulators are built as standalone GUI applications, are focused on human-readable visual simulation, and require a lot of computing resources.  Although with a bit of work you can get eg Gazebo working "headless" (no UI) from the command line, it is still slow to start and run on a computer with limited resources.  

### Status

Currently this is a sandbox for my personal development projects. I'm not actively seeking pull requests, and I make no guarantees about responding to them.  I'm maintaining it in a public repo in case anyone else finds it useful.
