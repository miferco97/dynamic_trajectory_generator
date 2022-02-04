## Dynamic Traj Generator

TODO: 

  - [x] Remove ROS stuff
  - [x] PLOT 2D and 3D plots at the same time
  - [x] yamlcpp installation
  - [x] 3D plotting
  - [ ] check Python Version for Matplotlib plotting purposes
  - [ ] Plotting only in tests?
  - [ ] Testing with Gtest
  - [ ] Relate Trajectory Modifiers with uav speed and position **!! IMPORTANTE**
  - [ ] Expand Time when modifiers are applied to mantain speed and acceleration constraints.
  - [ ] Create Dynamic Segments

Bugs known:
  - [ ] When dynamic waypoints are too close the previous gaussian modifies the actual waypoint trajectory

Dependencies:
- YamlCPP (```$ sudo apt install libyaml-cpp* ```)
