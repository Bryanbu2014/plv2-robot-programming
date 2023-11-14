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

`mako-render` will be placed into `.venv/bin/mako-render`. The makefile in this project requires that `mako-render` command is available in yout `PATH` when generating the maps, so make sure you call `make` later like this if you installed `mako` via `pip` by replacing `DIRECTORY_WHERE_MAKO_RENDER_IS`:

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
world_5_5.sdf
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


## Challenge 1a

```sh
./play world_1_1.sdf
```

Drive to the red wall as close as you can get and stop without colliding with the wall.

Some questions which could help you along:

- how would you set the speed of the robot?
- which control flow concept would you use to solve this problem? (e.g., `while`, `if-else`, etc)
- what is the minimal value of the LDS at which you should stop?
- does your robot stop at a different point every time you run your program? If yes, what could be the reason?

## Challenge 1b

Your robot should stop *smoothly* by decelerating at a given maximum acceleration. 

- how can you gradually decrease your velocity?
- look at [this feedback loopHint](https://commons.wikimedia.org/wiki/File:Ideal_feedback_model.svg).
  - what is your input, what is your output? 
  - what does the green + sign resemble?
  - what can `A` and `B` do?

## Challenge 1c

Your robot should always move *smoothly*. Also implement gradual acceleration. 

## Challenge 2a

```
./play world_1_1.sdf
```
Rotate 90 degrees counter-clockwise. The rotation movement must be *smooth* like in the last challenge. Only use the laser distance sensor data. If you cannot do an exact 90 degree rotation, that is fine.

Formulate a control equation for your robot similar to the last challenge.

- how can you calculate your rotation in degrees by only using laser distance sensor data?
- the laser distance sensor data will probably have some noise. How can you alleviate the noise?

## Challenge 2b

Drive to the red wall, and stop at a safe distance. Then rotate counter-clockwise and drive close to the wooden wall and stop. 

- this time we have to describe a more complex behavior than the last challenge. What is a useful tool to describe the behavior of your robot?
- do you know how to implement a state machine in Python?
- what is a safe distance where you can rotate without colliding with the red wall? Hint: What is the turning radius of the robot? [This may help](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#data-of-turtlebot3-waffle-pi).


## Challenge 3

```
./play world_1_1.sdf
```

You probably noticed that the laser distance sensor is noisy and driving while measuring can cause additional noise.  The challenge is the same, but instead of using laser distance data use the position and orientation published in `/odom`.

- every cell on the grid is 1m x 1m
- the topic `/odom` contains both the position and the orientation of the robot
- to convert a quaternion to Euler angles, you can use `from transforms3d.euler import quat2euler`. [API reference for `quat2euler`](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#quat2euler) could help.


## Challenge 4

```
./play world_2_2.sdf
```

Drive to the cell with the red wall and stop without any collision.

- how can you differentiate between wooden, white, and red walls?
- how do you know in which cell you are?
- how do you know to which cell you can/want to drive?


## Challenge 5

```
./play world_5_5.sdf
```

Touch the red wall in the shortest time as possible. Do not touch any other wall.

When you are finished, also try with another 5x5 world using `make clean; make`.

Optional: Try `world_9_9.sdf`.

## Challenge 6

Like challenge 5, but touch the cube box before touching the red wall.

When you touch it the time will be printed like:

```
[Msg] Model [...] started touching [tb3] at 1 867000000 seconds
```

which means 1.867 s.

Optional: How much calories per portion does the chocolate have?

# Multiple map generation of the same size for competition setting

In a competition setting the participants are ranked according to their completion time in challenges 5 and 6. For a competition a set of random maps of the same size are generated and each participant's robot is timed in each labyrinth and the end score is based on the average of multiple runs.

`make competition` generates multiple maps for the aforementioned competition setting. An example set is generated by the continuous integration script. These maps are available as artifacts under download symbol, `Previous Artifacts`, `build-competition-maps`.
