# Turtlebot3 maze challenges

## Setup

Install the Python library `mako`. The project uses the `mako-render` tool. First check if this tool is available on your `PATH`:

```
which mako-render
```

This prompt outputs the *full* path of the command. If the command cannot be found, then the output will be *empty*.

If not available, then you can install it in a virtual Python environment:

```
virtualenv venv
. ./venv/bin/activate
pip install mako
```

`mako-render` will be placed into `.venv/bin/mako-render`. The makefile in this project requires that `mako-render` command is available in your `PATH` when generating the maps, so make sure you call `make` later like this if you installed `mako` via `pip` by replacing `DIRECTORY_WHERE_MAKO_RENDER_IS`:

```
PATH=$PATH:DIRECTORY_WHERE_MAKO_RENDER_IS make
```

Clone the repository and create worlds and models:

```
cd tb3-maze-challenges
make
```

Then you should see the world files:

```
$ ls
...
world_1_1.sdf
world_2_2.sdf
...
```

If you have any problems, try making the project from scratch after cleaning:

```
make clean
make
```

# Challenges

Write programs which solve the following challenges. You can use [Turtlebot3 node template](https://mygit.th-deg.de/gaydos/tb3-ros2-template/-/blob/master/tb3.py) as a starter.

## Challenge 0

Play the challenge using the following command. Note that the first invocation of the simulator downloads some 3D models which may take about 60 seconds.
```sh
./play world_1_1.sdf
```

How much are the minimal distances to the walls? Use the laser distance sensor (LDS).

⚠️ `./play` sets `GAZEBO_MODEL_PATH` for the simulation models used in the challenges. Executing only `gazebo --verbose world_1_1.sdf` is not sufficient.

## Challenge 1

```sh
./play world_1_1.sdf
```

### a

Drive to the wall in front of the robot as close as you can get and stop without colliding with the wall.

Some questions which could help you along:

- how would you set the speed of the robot?
- which control flow concept would you use to solve this problem? (e.g., `while`, `if-else`, etc)
- what is the minimal value of the LDS at which you should stop?
- does your robot stop at a different point every time you run your program? If yes, what could be the reason?

### b

Your robot should stop *smoothly* by decelerating according to the distance.

- how can you gradually decrease your velocity?
- look at the feedback loop [here](https://commons.wikimedia.org/wiki/File:Ideal_feedback_model.svg).
  - what is your input, what is your output?
  - what does the green + sign resemble?
  - what can `A` and `B` do?
- the laser distance sensor data will probably have some noise. How can you alleviate the noise?

### c

The start motion of the robot must also be *smooth*.

## Challenge 2

```
./play world_1_1.sdf
```

### a

Rotate 90 degrees counter-clockwise without translation by *using the laser distance sensor data*. The rotation movement must be *smooth* like in the last challenge. If you cannot do an exact 90 degree rotation, that is fine.

<!--
Formulate a control equation for the rotation similar to the last challenge.

Hints:
- how can you calculate your rotation in degrees by only using laser distance sensor data?

-->

### b

Drive to the wall in front of you, and stop at a safe distance. Then rotate like in the previous subchallenge and drive close to the wall in front of you and finally stop.

- this time we have to describe a more complex behavior than the last challenge. What is a useful tool to describe the behavior of your robot?
- do you know how to implement a state machine in Python?
- what is a safe distance where you can rotate without colliding with the wall in front of you? Hint: What is the turning radius of the robot? [This may help](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#data-of-turtlebot3-waffle-pi).


## Challenge 3

```
./play world_1_1.sdf
```

You probably noticed that the laser distance sensor is noisy and driving while measuring can cause additional noise. The challenge is similar, but instead of using laser distance data use the position and orientation published in `/odom`.

Drive 15 cm ahead, rotate 90 degrees *clockwise* instead of counter-clockwise in the previous challenge, drive 15 cm ahead again and finally stop.

- the topic `/odom` contains both the position and the orientation of the robot. Robot's origin is in the middle of its driving axis. You can visualize the origin of the robot by activating `Wireframe` and `Link Frames` in `View` menu of Gazebo.
- to convert a quaternion to Euler angles, you can use `from transforms3d.euler import quat2euler`. [API reference for `quat2euler`](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#quat2euler) could help.
- you may run into problems when you try to solve the challenge on a physical robot. Solution hints:

  - the odometry of the physical Turtlebot3 [cannot be reset](https://github.com/ROBOTIS-GIT/turtlebot3/issues/750#issuecomment-860371676) compared to simulation.
  - the initial pose of the robot in Gazebo is well-defined, but the pose of the physical robot depends on the inertial measurement unit (IMU) and thus its pose relative to the magnetic field of the earth. You should place your robot with the exact pose (translation and orientation) like in simulation, then use the initial pose of `odom` as your starting pose which you can then use to create your own relative coordinate systems.
  - a coordinate system in ROS is called *frame*.
  - the robot's ground contact frame is called `base_footprint`. You can visualize the `odom` and `base_footprint` frames of the robot by running `rviz2`, adding a `TF` visualization, activating `Show Names` when the robot is running, i.e., publishing data.
  - for converting between frames and adding your own frames like *maze* frame you can use [tf2](http://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html) and the *intro to tf2* tutorial. If you prefer a *from scratch* approach, implement the conversions between frames yourself.
- every cell on the grid is 1m x 1m


## Challenge 4

```
./play world_2_2.sdf
```

Drive to the rightmost and topmost cell (marked with the red wall) and stop without any collision. However *do not assume* that the target cell is marked with the red wall, so do not use any color measurement.

Hints:

- how do you know in which cell you are?
- how do you know to which cell you can/want to drive?
- use two coordinate systems (aka *frames*) to describe the rotation of the robot relative to the maze: (1) *maze* (2) *robot* and implement the following functions:

```python
def robot_to_maze(rotation: float) -> float:
  """Transforms a rotation (in z axis) in the robot frame to a rotation in maze frame"""
  ...
def maze_to_robot(rotation: float) -> float:
  """Inverse of `robot_to_maze`"""
  ...
```
<!--
- use the previous functions to program something similar to:
  -
-->


## Challenge 5

```
./play world_3_3.sdf
```

Touch the rightmost and topmost wall (marked red) in the shortest time as possible. Do not touch any other wall. Similar to the previous question, do not assume that the wall is red.

When you touch the read wall the time will be printed like:

```
[Msg] Model [...] started touching [tb3] at 1 867000000 seconds
```

which means 1.867 s.


When you are finished, also try with another 3x3 world using `make clean; make`.

Optional: Try the with a larger world.

## Challenge 6

```
./play world_3_3_box.sdf
```

Like challenge 5, but touch the cube box before touching the red wall.


# Multiple map generation of the same size for competition setting

In a competition setting the participants are ranked according to their completion time in challenge 5. For a competition a set of random maps of the same size are generated and each participant's robot is timed in each labyrinth and the end score is based on the average of multiple runs.

`make competition` generates multiple maps for the aforementioned competition setting. An example set is generated by the continuous integration script. These maps are available as artifacts under download symbol, `Previous Artifacts`, `build-competition-maps`.
