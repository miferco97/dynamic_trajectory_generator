## Dynamic Traj Generator


Library for generating dynamic trajectories that changes over the time.





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


### Test example

Compile code:

```bash
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON && make
```

Run test from root directory:

```bash
./build/tests/dynamic_trajectory_generator_trajectory_test 
```

Plot results from root directory:

```bash
python3 tests/plot_results.py 
```