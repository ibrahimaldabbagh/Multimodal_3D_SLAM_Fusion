# quadmmwave_slam

4‑Directional mmWave Radar SLAM (ICP‑based) with IMU aid, robust ICP + RANSAC, adaptive voxel filtering, keyframes, and probabilistic (log‑odds) 2D mapping.

## Build
```bash
colcon build --packages-select quadmmwave_slam
source install/setup.bash
```

## Run
```bash
ros2 launch quadmmwave_slam quadmmwave.launch.py params:=/path/to/quadmmwave_params.yaml
```
