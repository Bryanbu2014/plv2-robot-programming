# PLV 2 - Robot Programming

## Contents in this Repository

- [src](/src/): Source code for the project
- [tb3-maze-challenges](/tb3-maze-challenges/): Files for simulation

# Turtlebot3 ROS2 template

A minimal template for controlling Turtlebot3

## Setup

Open three terminal windows for:

1. running your ROS node
2. writing code
3. launching & monitoring Gazebo

### 1st window - running your ROS node

```sh
git clone https://mygit.th-deg.de/gaydos/tb3-ros2-template && cd tb3-ros2-template
```

Then you can run your ROS node:

```sh
# Load your ROS environment, e.g.:
. /opt/ros/rolling/setup.sh

python tb3.py  # or python3 in Ubuntu
```

This should at least output:

```
waiting for messages...
```

If the robot is already brought up (in simulation or real), then you should also get `LaserScan` messages:

```
Distances:
⬆️: 0.5624107122421265
⬇️: 0.3894226849079132
...
```

### 2nd window - writing code

Open `tb3.py` with your your favorite text editor, e.g., `vim`, `Visual Studio Code (Code OSS)`

### 3rd window - launching & monitoring Gazebo

```sh
# Load your ROS environment, e.g.:
. /opt/ros/rolling/setup.sh

# Simulate a world, e.g:
gazebo --verbose WORLD_FILE.sdf
```

Should output:

```
Gazebo multi-robot simulator, ...
...
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
...
[Msg] Loading world file [...]
...
```

And if you open a world with TB3 or add it manually:

```
[INFO] [1608802634.071946263] [gazebo_ros_node]: ROS was initialized without arguments.
...
```

After TB3 is instantiated, you should see `LaserScan` values in the first window

## Notes

- In Gazebo, use `CTRL+r` on Gazebo to restart the simulation. This way you do not have to close and open Gazebo
- Use `CTRL+c` to kill a command line process, e.g., your ROS program or non-responsive Gazebo
- If you get the following error:

  ```
  [Err] ... EXCEPTION: Unable to start server[bind: Address already in use]. There is probably another Gazebo process running.
  ```

  then choose another port for Gazebo, e.g.:

  ```
  $ GAZEBO_MASTER_URI=:12346 gazebo --verbose WORLD.sdf
  ```

- If you do not use the simulation, then close Gazebo to leave the processor resources for others on the same machine. If the system is slow, `htop` may show you the reason. `htop` shows who the active processes belong to and processor usage.
- You can kill a process with process id 12345 using `kill 12345`. The process id is shown in the `PID` column in `htop`. Alternatively `killall PROCESS_NAME`, e.g., `killall gzserver`.
- `tools/` folder contain convenience scripts, e.g.,

  - `stop-tb3.sh`: to stop the robot manually
  - `tb3-teleop.sh`: teleoperation using keyboard
