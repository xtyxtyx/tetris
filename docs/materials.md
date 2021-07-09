---
title: Materials
about: materials
---

## Code
The code is available on [GitHub](https://github.com/jonathan-j-lee/tetris).

Launch files written for this project include:
1. [`ar_tracking.launch`](https://github.com/jonathan-j-lee/tetris/blob/master/src/tetris/launch/ar_tracking.launch): Runs two `ar_track_alvar` nodes (one for individual tile markers, the other for the board as a bundle).
1. [`baxter_moveit_headless.launch`](https://github.com/jonathan-j-lee/tetris/blob/master/src/tetris/launch/baxter_moveit_gui.launch): Runs the MoveIt planner and Baxter joint trajectory action server without launching an RViz GUI.
1. [`baxter_moveit_gui.launch`](https://github.com/jonathan-j-lee/tetris/blob/master/src/tetris/launch/baxter_moveit_gui.launch): Like `baxter_moveit_headless.launch`, but with the GUI and without trajectory execution.
1. [`baxter_state_publisher.launch`](https://github.com/jonathan-j-lee/tetris/blob/master/src/tetris/launch/baxter_state_publisher.launch): Publishes the robot's links and joints to TF. You won't need to use this unless Baxter starts with its links disconnected.
1. [`tetris.launch`](https://github.com/jonathan-j-lee/tetris/blob/master/src/tetris/launch/tetris.launch): Invokes the main Tetris solver and pick-and-place module. It also runs the first three launch files above, so it suffices to launch from this file only.

<br>
## Laser Cut File
The laser cut file for the board and pieces can be found [here]({{ site.baseurl }}/assets/adobe_illustrator/Tetris-Puzzle-AR-Tag.ai).
