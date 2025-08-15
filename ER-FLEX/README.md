# ER-FLEX progress documentation
## Network setup
Both robots  are configured to connect to a network with the following credentials: SSID: HR_department, password: TUTELrobot3.
There are two web interfaces for each robot, see table below:

| Robot | Hostname | ER interface IP | MiR interface IP |
| - | - | - | - |
| Asimo | ASIMO-er-flex-250 | 192.168.0.102 | 192.168.0.101 |
| Bane | BANE-er-flex-250 | 192.168.0.112 | 192.168.0.111 |

The credentials for loging to **all** of the interfaces are the same: username: admin, password: admin 

### Networking troubleshooting
TODO @nkymark

**MiR interface**
Dedicated to the MiR moblie platform. Used for creating maps, defining poses for robot, defining charginig dock, managing MiR features (e.g. sounds).

**Enabled Robotics interface**
For managing the robot whole. Creating programs for the robot, creating missions, remotely accessing the UR teach pendant.

## Starting the robot
For the robot to be fully started, both MiR cart and UR5 must be on.
- Turn on them MiR by pressing the button in the lower left corner of the back side
- Turn on the UR5 arm by pressing button on the teach pendant. Wait for the robot to boot and then activate the arm using the teach pendant.
- Once the button on the back side starts flashing blue, press it to release the parking brake.
Manufacturers documentation [here](original_documentation/3.1.0-er-flex-250-user-manual.pdf)

<mark>Important:</mark> The cart won't move unless the arm is in its **Safe Home position**. 

## Changes made to the robot
#### Bane
New TCP of the UR5 has been created, accounting for the mounted gripper. This TCP has been set as default to the 'default' installation of the robot. This has to be taken into account when programming arm movements with the blocks in the ER web interface, as the default TCP (Tool) for the movement blocks is 'Flange', which is different than the modified TCP. Instead, the TCP (Tool) **Robotiq_2_finger** should be used.

<mark>TL;DR</mark> When programming arm movements using blocks, set the ``Tool`` option to **Robotiq_2_finger**.
![alt text](resource/set_tool.png)

#### Asimo
none

## Creating a program
You can "write" the main program in the ER web interface using blocks. To interact with the gripper and command the UR5 to do things other than moving, the UR Event block can be used. The UR Events can be defined in the 'ability' script. Manufacturers documentation [here](original_documentation/ability-2.14.0-user-docs.pdf)

### Ability script
This is a script for managing the UR5 manipulator actions. It can be accessed via the Teach pendant. THe script *should* start automatically when switching to 'remote' mode.
EventNodes can be added to the script to execute manipulator tasks. The nodes can then be launched from the main program (blocks). The node can take arguments passed from the main program, it is important to set the correct datatype. If you are using more parameters, they should be passed in as a list.
![alt text](resource/event_node_params.png) ![alt text](resource/event_node_block.png)

## Running a program
To run the program switch the UR to 'remote' mode, wait for the 'ability' script to start, make sure the gripper is activated and then launch the main program.

### Two stations paletitzing program
This program is saved on Bane robot.
In this program, the robot drives to first station and detects pallet with workpieces using the camera mounted on the UR5. Then the robot moves above the pallet and sends its relative position as a parameter of UR Event. In the UR Event the robot calculates the positions of three workpieces and moves them into the pallet on the MiR platform. Then drives to second position, where it detects a pallet to unload the workpieces and unloads them. Then the robot returns to the first position. The UR Events used are  ``Paletite`` and ``Unload``. These must be included in the ability script for the program to run.

#### Running in different enviroment
You will need to define two safe positions for the MiR, the UR5 will try to detect the pallet to the right side of the robot. To change that, edit 'A_calibrate' and 'B_calibrate' waypoints in the program.



## Encountered problems (and solutions)
- **Teach pendant in the ER interface won't open** If a window showing an error opens after clicking the pendant icon, make sure the UR5 is on and active. If nothing happends after clicking, refresh the page. If that doesn't help, clear the browser data, (cookies, etc.) and try again.

- **Ability script doesn't start after switching to remote mode** 
We don't know what exactly causes the problem. Possible solutions, that sometimes work: 
1. Wait for a bit (up to a minute), sometimes it just takes longer.
2. Switch to 'local' mode, start the script so the variables appear, stop the script and then switch to 'remote'.
3. Switch to 'local' mode, star the script, let it running and switch to 'remote'. Wait for the script to stop and start again automatically. The main program (blocks) will not work when the script is started manually, you have to wait for the script to restart.
4. Reboot the UR5 and try again.

- **MiR mobile platform won't drive** Check if the MiR is paused in the MiR web interface. Check if the UR5 manipiulator is in 'Safe Home' position. Check if any of the ``Emergency-stop`` buttons is pressed. Check if something is near the robot (20 cm) and can be detected by the lidar, for example the teach pendant cable.

##  Starting 'ability' script problem
We are having problems with the ER-Flex robot. We expect the “ability” script of the UR manipulator to be started and enable the main program to use the defined EventNodes when switching to ‘remote’ mode. However the behavior of the script is inconsistent. Often after switching to ‘remote’ mode, the ‘ability’ script takes up to a minute to start, or doesn’t start at all.
We made sure the manipulator is fully activated and can be operated in ‘local’ mode, also that the safety stop is released.
We added a total of four EventNodes to the ‘ability’ script. The script itself can be seen below.
We suspected the problem might be caused by the length of the script, but it kept occurring even after deleting parts of the program (making it shorter) and reducing the number of EventNodes.
This Issue keeps appearing unpredictably. Sometimes switching between ‘local’ and ‘remote’ mode a few times helps to start the script. Starting the script in ‘local’ mode and then switching to ‘remote’ also helped.
However we didn’t find any persistent cause of the issue, nor a reliable solution.
