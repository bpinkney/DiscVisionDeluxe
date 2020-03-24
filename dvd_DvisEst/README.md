# DiscVisionDeluxe Vision Capture and Initial State Estimator

Sequence of dvd_visEst routine:
- Observe high-fps video feed
- Run AprilTag detector on each frame
- Upon detection, use single/multiple tag detections to produce a single position and orientation measurement
- Use pos/orient measurements to produce a KF-style disc state estimate
- Update state estimate throughout disc flight to obtain confident states and derivatives
- Serve finalized initial state to dvd_DfisX and dvd_DgrafX

To build with CMAKE:

Linux:
<add lib deps and build steps here>

To build using CMake:
``` bash
cd build/
cmake ..
make
```



Windows:
<add lib deps and build steps here>

