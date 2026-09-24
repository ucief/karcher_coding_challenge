# Kärcher Robotics Coding Challenge
**Path Length - Cleaned Area - Traversal Time**

A cleaning robot has driven through a building and recorded where it went. Based on the recording, the
robot's footprint, and the geometry of its cleaning gadget this repository calculates three things about the run: 
1. how far it drove, 
2. how much floor it actually cleaned, and 
3. how long it would take to drive that path at a realistic speed.

The results can be viewed in the 
[interactive visualization](https://ucief.github.io/karcher_coding_challenge/).
In the visualization different layers can be added or removed, by left-clicking the layer on the right.
You can zoom in, or move the plot with the controls on the top right. You can see the trajectory values by hovering over the graph.

## Getting Started

### Build and test C++

Requirements: CMake 3.16 or newer, a C++17 compiler (GCC or Clang on Linux),
Make, and Boost 1.75 or newer with the JSON library.
On Ubuntu/Debian, install the build tools with:

```sh
sudo apt update
sudo apt install build-essential cmake libboost-json-dev
```

Run these commands from the project root (the folder containing `CMakeLists.txt`):

```sh
# Configure a Debug build with symbols for debugging.
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug

# Compile the library and test executables.
cmake --build build

# Run the tests and show details if a test fails.
ctest --test-dir build --output-on-failure
```

Run the analysis on a recording:

```sh
./build/analyze_path data/short.json
```

Optionally export the computed trajectory for plotting:

```sh
./build/analyze_path \
    data/short.json \
    results/trajectory.csv \
    results/cleaned_area.csv \
    results/results.json
```

The CSV contains world positions, heading in radians, and outgoing-segment
curvature and speed. Undefined headings and the final row's segment values are
blank. The input filename is required; use `--help` for usage.

Validation includes analytic straight sweeps, repeated coverage, arbitrary
orientation, and circular sectors/annuli with holes.

### Running Python Code:
If you do not want to install python dependencies, you can check out the plot [here](https://ucief.github.io/karcher_coding_challenge/).

Install Requirements
```
python3 -m venv .venv
source .venv/bin/activate
pip install -r visualization/requirements.txt
```

Then you can run the code to create the interactive plot:
```
python3 visualization/visualize.py
```
Make sure, that you have exported the data before using the C++ script.

## How it works:
### Preprocessing
If the robot is not moving, samples consist mainly of measurement noise. If wanted, they can be removed in the preprocessing. By default, the `PREPROCESSING_MINIMUM_DISTANCE_M` is set to 1 cm. Points, that are closer than 1 cm are removed from the path.
I determined the best parameters to use with an initial analysis of the data in a jupyter notebook `./visualization/visualize_data.ipynb`.
If these points should be kept in the trajectory, set the `PREPROCESSING_MINIMUM_DISTANCE_M == 0.0`.

### Heading and Curvature Estimation

Since the input path contains positions only, the robot heading is estimated from the local path direction. Around each waypoint, the estimation window is expanded until its endpoints are separated by a minimum spatial distance. The heading is then computed with `atan2`. This way, if the robot is not moving forward, the window size is increased, until the robot has moved at least by `HEADING_MINIMUM_DISTANCE_M` reducing sensitivity to small noisy position changes.

Curvature is estimated using the same principle:
$$
\kappa \approx \frac{|\Delta \theta|}{\Delta s},
$$

with the heading difference and a minimum spatial baseline defined by `CURVATURE_MINIMUM_DISTANCE_M`. Estimates that cannot be determined reliably, because the minimum distance can not be reached are stored as `NaN`.

### Calculating the Cleaned Area

The cleaned area is approximated by sweeping the cleaning-gadget segment along
the estimated robot trajectory. The outer positions of the cleaning gadget are transformed to the world coordinates using the current pose and heading of the robot. 

Motion (position and heading) between consecutive poses is interpolated linearly with a maximum step of 2 cm translation or 2° rotation. 
For every interpolation step, the previous and current gadget outer positions form a polygon.

All sweep polygons are combined using a geometric union before calculating their
area. Therefore, regions that are traversed multiple times are counted only once.
The resulting `MultiPolygon` also preserves holes and disconnected cleaned regions. It is the same Polygon, that is displayed in the [visualization](https://ucief.github.io/karcher_coding_challenge/).

#### What Error is induced:
The rotation introduces a discretization error because the gadget endpoints of the robot follow a circular motion, that is approximated by straight lines in the polygon.

By limiting the heading change to 2° and the error can be approximated. 
For a simple radial gadget rotating about the robot ICP, the exact swept area for one angular step $\Delta\theta$ is

$$
A_{\text{true}}
=
\frac{1}{2}\left(r_2^2-r_1^2\right)\Delta\theta
$$

while the polygonal approximation gives

$$
A_{\text{poly}}
=
\frac{1}{2}\left(r_2^2-r_1^2\right)\sin(\Delta\theta).
$$

The relative error is therefore

$$
\frac{A_{\text{true}}-A_{\text{poly}}}{A_{\text{true}}}
=
1-\frac{\sin(\Delta\theta)}{\Delta\theta}.
$$

For a maximum angular step of $2^\circ$,

$$
\Delta\theta
\approx
0.0349\ \text{rad},
$$

which gives

$$
1-\frac{\sin(\Delta\theta)}{\Delta\theta}
\approx
0.00020
=
0.020\%.
$$

For small angular steps, the error decreases approximately with $\Delta\theta^2$.


A separate source of uncertainty is the assumption of linear interpolation between recorded poses, since the exact robot motion between samples is not available.

Further, heading is approximated, by averaging over the heading window to compensate for measurement noise.
Therefore, if the robot is turning much faster, than it is moving, the changes in heading might be underestimated which can change the resulting Area.  

## What the tests cover:
The test suite covers:

- **JSON input parsing and validation**
  - valid recordings, preserved waypoint order and duplicates
  - missing or malformed fields, invalid point dimensions/types, non-finite values
  - robot polygon and cleaning-gadget geometry validation
  - loading recordings from files and handling invalid paths

- **Path processing**
  - path-length calculation, including duplicate points and tracking jitter
  - trajectory preprocessing and invalid filter parameters
  - heading estimation with adaptive spatial windows, including path boundaries and undefined headings

- **Curvature and traversal time**
  - straight and curved trajectories
  - angle wraparound at ±π
  - undefined curvature for zero-length or invalid-heading segments
  - speed-model breakpoints and traversal-time calculation

- **Coordinate transformations**
  - translation and rotation from robot coordinates to world coordinates

- **Cleaned-area computation**
  - straight sweeps with analytically known areas
  - overlapping/retraced paths without double-counting
  - rotated and translated trajectories
  - disconnected valid regions
  - rotational sweeps, holes, and angle wraparound

- **Integration/regression test**
  - loads the supplied `short.json`
  - checks the expected input dimensions
  - verifies the cleaned area against an independent raster reference (~8.088 m²)

## Assumptions
- Measurements are taken directly for `base_link` $\rightarrow$ no transformation between measurements and `base_link`
- The robot moves in a straight line between two measured positions.
- The robot can be approximated by a unicycle model. This means, no lateral velocity or in other words, the robot always drives forward and the direction of movement is only in `x`-direction in robot frame.
- Path coordinates are ordered cronologically.
- The traversal time estimation assumes, that the robot speed can be achieved instantly. It uses the speed model from the challenge description, but does not take acceleration into account. The real robot has acceleration limits, that will increase the real execution time.

## If I had more time:
- Clean up the speed legend, that currently is displayed over the layers
- Add a dedicated parameter file and expose parameters to command line
- **Uncertainty handling:** Propagate uncertainty from the recorded positions into heading, curvature, traversal time, and cleaned-area estimates instead of using fixed thresholds only.
- **Extended input validation:** Add checks for self-intersecting robot polygons, invalid gadget placement, coordinate conventions, and unrealistic trajectory jumps.
- Add rendered latex formulas to view Readme in Github