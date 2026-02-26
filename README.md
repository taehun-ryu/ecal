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
  <!-- <a href='https://openaccess.thecvf.com/content/CVPR2025/papers/Lee_HUSH_Holistic_Panoramic_3D_Scene_Understanding_using_Spherical_Harmonics_CVPR_2025_paper.pdf'><img src='https://img.shields.io/badge/Paper-CvF-blue'></a> -->
  <a href='https://vision3d-lab.github.io/corner2tag/'><img src='https://img.shields.io/badge/Project-Page-gr'></a>
</div>

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
