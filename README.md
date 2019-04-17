ROSflight Unity SIL Simulation
==============================

Infrastructure for running a ROSflight software-in-the-loop (SIL) simulation with Unity (see [rosflight_unity_simulator](https://github.com/plusk01/rosflight_unity_simulator)).

## Getting Started

#### ROS Setup

1. Clone this repo into your `catkin_ws`.
2. Clone [rosflight](https://github.com/rosflight/rosflight) into your `catkin_ws` and `git submodule update --init --recursive`.
3. `catkin build` or `catkin_make`

#### Unity Setup

1. Install [Unity 2018.2.7f1](https://forum.unity.com/threads/unity-on-linux-release-notes-and-known-issues.350256/page-2#post-3662605) (on Ubuntu).
2. Clone [rosflight_unity_simulator](https://github.com/plusk01/rosflight_unity_simulator) into your favourite location.

#### Flying Manually

0. Clone [`rosflight_joy`](https://github.com/rosflight/rosflight_joy) into your `catkin_ws` and build.
1. After sourcing your `catkin_ws`, run `rosrun rosflight_unity rosflight_unity` to start the ROSflight Unity SIL.
2. Press the 'play' button in the `rosflight_unity_simulator` Unity project.
3. Run `rosrun rosflight rosflight_io _udp:=true`
4. Plug in your compatible (e.g., Taranis) RC transmitter.
5. Run `rosrun rosflight_joy rc_joy RC:=/rosflight_unity/rc_in`
6. Set the [mixer to 2](http://docs.rosflight.org/en/latest/user-guide/hardware-setup/#motor-layouts): `rosservice call /param_set MIXER 2`
7. Write the autopilot parameters to memory: `rosservice call /param_write` (params will be written to file, in whatever directory you started the `rosflight_unity` node)
8. Calibrate the IMU: `rosservice call /calibrate_imu`
9. Throttle stick down and to the right to arm (down and to the left to disarm).
10. Fly!

