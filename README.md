# modelcars

### compiling

* Use `catkin build --profile <profile>` to compile on a PC where `profile` is `kinetic_x86` or `indigo_x86`.

* To compile for arm run `crosscompile/install.sh` (once) and use `catkin build --profile indigo_arm`. Make sure you have installed the odroid sdk and compiler to `/opt/odroid-x2/` and installed all other dependencies mentioned at the [Wiki](https://github.com/AutoModelCar/AutoModelCarWiki/wiki/Cross-compile).

* You can set your favourite profile with `catkin profile set <profile>`.

### testing with recorded data

* To test Your code, add Your node to `test/launch/launch.launch` and launch with `roslaunch test launch.launch`. This will launch Your node, a log_player and rviz. The following topics are published by the player:
    * `/usb_cam/image_raw` - The raw image
    * `/odom` - Odometry data
    * `/path` - The Ground Truth trajectory (only available for artificial data)

* If You're going to generate new logs:
    * Use `roslaunch logging log_recorder.launch filename:='f'` to record data on the robot and store it in the file 'f'.bag.
    * Alternatively, build artificial data by hand in `logging/src/log_generator.cpp` and generate it using `roslaunch logging log_generator.launch`.
    * Use `roslaunch logging log_player.launch` to play back a log file.

