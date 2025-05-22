# ostem
OStem (Outdoor Stembot) is a custom designed outdoor Autonomous Guided Vehicle powered by ROS2. Uses 
GNSS RTK to obtain real time coordinates with centimeter level accuracy and uses this with mapviz and Nav2
to perform autonomous navigation to target lattitudes and longitudes.

## Features:
- Implements an similar robot design as URDF for visualization and simulation
- Contains a GPS synced world (sonoma raceway) with robot that can be simulated in gazebo
- Includes an outdoor navigation package that is tested to work in simulation

## TODO:
- [x] Develop URDF for the robot
- [x] Configure and develop packages for simulation of outdoor robot with GPS
- [x] Use the sim to develop and test localization packages
- [x] Use the sim to develop and test navigation packages
- [x] Plan the hardware components that will be used
- [ ] Develop bringup package for the robot hardware
- [ ] Test the hardware robot with localization and navigation package