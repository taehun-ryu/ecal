<p align="center">

  <h1 align="center">From Corners to Fiducial Tags: Revisiting Checkerboard Calibration for Event Cameras</h1>

  <p align="center">
    <a href="https://taehun-ryu.github.io/" rel="external nofollow noopener" target="_blank"><strong>Taehun Ryu</strong></a>
    ·
    <a href="https://kang-changwoo.github.io/" rel="external nofollow noopener" target="_blank"><strong>Changwoo Kang</strong></a>
    ·
    <a href="https://vision3d-lab.github.io/" rel="external nofollow noopener" target="_blank"><strong>Kyungdon Joo</strong></a>
  </p>

<div align='center'>
  <a href='https://vision3d-lab.github.io/corner2tag/'><img src='https://img.shields.io/badge/Project-Page-gr'></a>
  <a href='https://taehun-ryu.github.io/corner2tag/'><img src='https://img.shields.io/badge/Docs-MkDocs-0ea5e9?style=flat-square&logo=materialformkdocs&logoColor=white' alt='Documentation'></a>
</div>

> This repository is the official implementation of the paper "From Corners to Fiducial Tags: Revisiting Checkerboard Calibration for Event Cameras".

## News
<!-- :tada: celerbrate -->
<!-- :rocket: major release -->
<!-- :sparkles: new feature / implementation -->
<!-- :bug: bug fix -->
- 26.02.23. :tada: corner2tag was accepted to CVPR 2026! Final recommendation: 5-5-6.

## TODO
- [ ] Checkerboard calibration pipeline
- [ ] AprilTag calibration pipeline
- [ ] Docker support for checkerboard calibration
- [ ] Docker support for AprilTag calibration
- [ ] Inivation `.aedat4` to `h5` converter
- [ ] ROS1 `dvs_msgs` to `h5` converter
- [ ] ROS1 `prophesee_event_msgs` to `h5` converter
- [ ] ROS2 `event_camera_msgs` driver support (direct `.db3` input)

ROS support note: Our current focus is **ROS2** `event_camera_msgs` development, and ROS1 support is provided through `h5` converters.

## How to use

For detailed setup and usage instructions, please refer to [the full documentation](https://taehun-ryu.github.io/corner2tag/).

## Acknowledgments

This work is built on several great research works, thanks a lot to all the authors for sharing their works.

- [Event Cameras, Contrast Maximization and Reward Functions: An Analysis [CVPR 2019]](https://github.com/TimoStoff/events_contrast_maximization)
- [Target-free Extrinsic Calibration of Event-LiDAR Dyad using Edge Correspondences [RA-L 2023]](https://github.com/wlxing1901/contrast-maximization-for-event-cameras)

## Citation
```latex
@InProceedings{ryu2026corner2tag,
    author    = {Taehun Ryu and Changwoo Kang and Kyungdon Joo},
    title     = {From Corners to Fiducial Tags: Revisiting Checkerboard Calibration for Event Cameras},
    booktitle = {IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
    month     = {},
    year      = {2026},
    pages     = {}
}
```
