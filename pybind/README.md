# dynamic_trajectory_generator_py — Python bindings for Dynamic Trajectory Generator

This package exposes the `DynamicTrajectory` API of
`dynamic_trajectory_generator` to Python.

## Installation

From the repository root:

```bash
pip install ./pybind
```

This builds the C++ library, its transitive dependencies
(`mav_trajectory_generation`, `nlopt`) and the pybind11 extension, and
installs them as a self-contained wheel under the `dynamic_trajectory_generator_py` package.

### Packaging details

- Build backend: `scikit-build-core` + CMake.
- The wheel includes the Python package plus the native extension
  (`_dynamic_trajectory_generator_cpp`).
- Shared-library dependencies built in-tree are installed alongside the extension.
- Runtime search path is set to `$ORIGIN`, so co-located shared libraries resolve
  without requiring manual `LD_LIBRARY_PATH` changes in normal use.

### Requirements

- A C++17-capable compiler.
- CMake >= 3.16.
- Eigen3 development headers.
- pybind11 (installed automatically via scikit-build-core if not present).
- NumPy >= 1.22.
- Python >= 3.8.

## Usage

```python
import numpy as np
from dynamic_trajectory_generator_py import DynamicTrajectory

traj = DynamicTrajectory()

initial_position = np.array([0.0, 0.0, 0.0])
initial_yaw = 0.0
waypoints = [
    np.array([0.0, 0.0, 10.0]),
    np.array([10.0, 0.0, 10.0]),
    np.array([10.0, 10.0, 20.0]),
]
max_velocity = 3.0

traj.generate_trajectory(initial_position, initial_yaw, waypoints, max_velocity)
traj.set_path_facing(True)

t_min = traj.get_min_time()
t_max = traj.get_max_time()

position, velocity, acceleration, yaw = traj.evaluate_trajectory(0.5 * (t_min + t_max))
```

## Local development

### Editable install

From the repository root:

```bash
pip install -e ./pybind
```

### Build with CMake and import from build tree

This repository mirrors the Python package into the CMake build directory, so you can
import directly from there while iterating on bindings:

```bash
cmake -S pybind -B pybind/build/dev
cmake --build pybind/build/dev -j
PYTHONPATH="$PWD/pybind/build/dev/python:$PYTHONPATH" 
```

And then in Python:

```python
from dynamic_trajectory_generator_py import DynamicTrajectory
```

## Testing

Install test dependencies:

```bash
pip install -e "./pybind[test]"
```

Run tests:

```bash
pytest pybind/tests
```

Notes:

- `test_dynamic_trajectory.py` includes smoke checks for import/export and constructor.
- `test_code_style_dynamic_trajectory_generator.py` runs style tools
  (`ament_flake8`, `flake8`, `ament_pep257`, `pydocstyle`) when available.
- If one of those style tools is missing or broken in the current environment, the test emits
  a warning and is skipped instead of failing.

## License

BSD-3-Clause.
