# From Corners to Fiducial Tags: Revisiting Checkerboard Calibration for Event Cameras

## Dependencies
- ceres
- hdf5
- opencv

## News
<!-- :tada: celerbrate -->
<!-- :rocket: major release -->
<!-- :sparkles: new feature / implementation -->
<!-- :bug: bug fix -->
- 26.XX.XX. :sparkles: Added Docker support and implemented converters from ROS1 `dvs_msgs` and Inivation `.aedat4` to HDF5.
- 26.XX.XX. :rocket: Released the checkerboard calibration code.
- 26.02.23. :tada: ecal is accepted to CVPR 2026! Final recommendation: 5-5-6.

## TODO
- [ ] AprilTag calibration pipeline
- [ ] Docker support for AprilTag calibration
- [ ] ROS1 `prophesee_event_msgs` to `h5` converter
- [ ] ROS2 `event_camera_msgs` driver support (direct `.db3` input)

ROS support note: Our current focus is **ROS2** `event_camera_msgs` development, and ROS1 support is provided through `h5` converters.

## Build
```bash
cmake -S . -B build
cmake --build build -j
```

## Run
Checkerboard:
```bash
./build/ecal_checkerboard_calibrate config/checkerboard.yaml
```
Output directory:
`results/checkerboard/run_YYYYMMDD_HHMMSS/`

Apriltag:
- planned

## Docker
Docker runs with GUI disabled by default (`ECAL_ENABLE_GUI=0`).
If display forwarding is configured, enable GUI with `ECAL_ENABLE_GUI=1`.

### Checkerboard
```bash
bash docker/run_checkerboard_docker.sh /absolute/path/to/your/data your_file.h5
```

Results are written to:
`results/checkerboard/run_YYYYMMDD_HHMMSS/` (under repository root)

Each run uses a new timestamped `run_*` folder to avoid overwriting previous results.

Enable GUI example:
```bash
ECAL_ENABLE_GUI=1 bash docker/run_checkerboard_docker.sh /absolute/path/to/your/data your_file.h5
```

### Apriltag (planned)
- planned

### Help
For full options and usage:
```bash
bash docker/run_checkerboard_docker.sh --help
```
