# modelcars

### compiling

* Use `catkin build --profile <profile>` to compile on a PC where `profile` is `kinetic_x86` or `indigo_x86`.

* To compile for arm run `crosscompile/install.sh` (once) and use `catkin build --profile indigo_arm`. Make sure you have installed the odroid sdk and compiler to `/opt/odroid-x2/` and installed all other dependencies mentioned at the [Wiki](https://github.com/AutoModelCar/AutoModelCarWiki/wiki/Cross-compile).

* You can set your favourite profile with `catkin profile set <profile>`.

