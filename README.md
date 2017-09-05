# modelcars

### Quick Start Guide

To start the localization, several things have to be (ros)launched, depending on what You want to do:

##### Regular Start

* `fisheye_camera_matrix camera_matrix_provider.launch`
    * publishes /usb_cam/camera_matrix which is needed to undistort the fisheye-image and to compute perspective projections
* `fisheye_camera_matrix undistorted_image_provider.launch`
    * publishes /usb_cam/image_undistorted
* `cps2 localization_publisher.launch`
    * publishes /localization/cps2/pose containing the cars pose in world frame

##### Debugging on Car

Alternatively, You may launch `cps2 localization_publisher_debug.launch` which launches all of the above. The debug version publishes some markers and writes the trajectory to a file. Use `cps2 rviz.launch` on a remote PC to make things visible.

##### Debugging on Desktop

Alternatively, just launch `cps2 test_localization_publisher` which launches all of the above plus a rosbag player.

### How does it work?

// TODO

### Parameters to set

All parameters are briefly explained in the launchfiles where they are supposed to be set. Here, we like to give a more detailed explaination:

// TODO

### Build Status
[![Build Status](https://travis-ci.org/mauriceAlthoff/modelcars/modelcars.png)](https://travis-ci.org/mauriceAlthoff/modelcars/modelcars.png)
