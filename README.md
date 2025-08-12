# turtlebot4-lab-tasks
Documentation of tasks constructed with turtlebot4 robots.

**Contents:** <br>
[**Setting up**](#setting-up) <br>
[Robot parameters](#robot-parameters) <br>
[Common issues (FAQ)](#common-issues-faq)<br>
[Reinstalling OS](#reinstalling-os)<br>
[Time synchronization](#time-synchronization)<br>
[**Basic use cases**](#basic-use-cases)<br>
[Teleoperation](#teleoperation)<br>
[BT Gamepad](#bt-gamepad)<br>
[**Camera use cases**](#camera-use-cases)<br>
[Color tracking](#color-tracking)<br>
[QR code commands](#qr-code-commands)<br>
[**SLAM**](#slam)<br>
[Generating a map](#generating-a-map)<br>
[Autonomous navigation between points](#autonomous-navigation-between-points)<br>
[Cooperating turtlebots PoC](#cooperating-turtlebots-proof-of-concept)


## Setting up
- Before powering the robot up by placing it on the dock, make sure the network is already up and running. If the robot tries to connect before the network is ready, it will fail and you will need to restart it. <br>
- **Lightring** indicates what is the robot doing. during the startup the LEDs spin. Do not make the robot do anything before it is ready (light stops spinning and it chimes) it *might* break it and reboot might be needed.
- On startup the lightring *will* turn yellow - indicating an error while connecting to Create3, however after 30 seconds the connection estabilished. Just wait for a bit.
- The robot is ready once the lightring is white and all 5 control LEDs are green.
- You can connect to the robot via SSH with username 'ubuntu' and password 'turtlebot4', this is the same for all of the robots. The IP adress is displayed on the screen onboard.

### Robot parameters
| Namespace | Discovery server ID | IP adress | Create3 frmw ver. | Working time sync | OS |
| --------- | ------------------- | --------- | ----------------- | ----------------- | -- |
| /bob | 2 | 192.168.0.220 | H.2.6 | NO | tutel |
| /bobek | 3 | 192.168.0.20 | H.2.6 | NO | tutel |
| /turtlebots/tutel | 1 | 192.168.0.134 | H.2.6 | YES | tutel |
| /turtlebots/chomik | 0 | 192.168.0.23 | H.2.6 | YES | tutel |

**Notes**

- The networks SSID is 'HR_department' and the password is 'tutelROBOT3'
- OS "tutel" refers to a particular version and configuration that prooved to work. The file is too big to be uploaded, hence is stored elsewere. More info in [Reinstalling OS](#reinstalling-os)
- Robots are using Discovery Server configuration, to reduce network traffic ([info here](https://turtlebot.github.io/turtlebot4-user-manual/setup/networking.html#simple-discovery)), to see ROS topics of the robots user PC must be configured with the right IP adresses and server IDs ([see here](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html#user-pc))

### Common issues (FAQ)
**ROS topics not visible**
Possible causes/solutions:
- Run ```ros2 topic list``` again, sometimes after startup the topics won't show immidiately.
- Check if the "COMM" LED onboard is green, sometimes the Create3 randomly disconnects and stops publishing to topics. Reboot the robot.
- The user PC might be configured wrong, try redoing the setup [see here](https://turtlebot.github.io/turtlebot4-user-manual/setup/discovery_server.html#user-pc) and make sure to enter correct values.

**How to stop the lidar?**
Call service:

```
ros2 service call /turtlebots/chomik/stop_motor std_srvs/srv/Empty {}
```

**Battery % showing 0?** Battery status is recieved from Create3, if it shows 0, Create3 isn't connected propperly. Reboot the robot.

**Lightring flashing red** Battery level is below 20 %.

**What network are robots connected to?** The networks SSID is 'HR_department' and the password is 'tutelROBOT3'.




### Reinstalling OS
Download the image of the OS (the one that worked for us, no guarantees for versions from the turtlebot web). Use the [RasPi imager](https://www.raspberrypi.com/software/) to write to the SD card. Do not customize any parameters. On startup the robot *will not* be in the accesspoint mode, it will try to connect to a netework with SSID "HR_department" It will also have its namespace set to */tutel*, it will be configured as Discovery Server and The discovery Server ID will be 1. Those parameters need to be changed accordingly. I Recommend not having any other turtlebots active when first starting the reinstalled one. The creditals for ssh are the same username: ubuntu, password: turtlebot4

### Time synchronization
## Basic use cases
### Teleoperation
For driving the robot using keyboard run:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/<namespace>
```
Make sure to use the right namespace for the robot.
### BT Gamepad
## Camera use-cases
### Color tracking
### QR code commands
## SLAM
### Generating a map
### Autonomous navigation between points
<mark> Important: </mark> <br>
For some reason, launching RViz, localization and navigation is only succesful **once after startup** if you end the process, to get it to run again you will *probably* have to restart the robot.
### Cooperating Turtlebots (proof of concept)


