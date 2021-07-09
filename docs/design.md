---
title: Design
about: design
---

## Design Criteria
In order to complete this task the robot had to:
* Find the correct location and orientation of a piece
* Pick up the piece with a vacuum gripper in a stable manner
* Compute the piece's solution on the board
* Place the piece onto its solution location in the correct orientation
* Don't hit the table or board while moving
* Have all the pieces fit together flush on the board
<br/><br/>

This project will allow us the robot to complete a full game of the Tetris puzzle.

![Completed puzzle]({{ site.baseurl }}/assets/images/completed-puzzle.png)

## Design Choices
* The pieces were sanded down to fit together more easily on the board.
* We chose to use computer vision to identify the type of tetris pieces and the board as well as its location and orientation. We use the left-hand camera to locate and identify the pieces (the head camera is not able to get a good view of the table).
* The vacuum gripper was chosen for pick-and-place because it is able to obtain a more stable hold on the thin pieces than the electric gripper. When attempting to grasp a piece, we move the gripper to some safe margin above the piece and use the sensor attached to the gripper to determine whether the vacuum has created a good seal. We then slowly iterate between lowering the gripper and querying for a good seal until we have successfully grasped the piece. This reduces the negative effects of any gripper positional error in the z direction and is a safe way to ensure we have grasped the piece.
* The `MoveIt` package is used for path planning to ensure that we avoid obstacles like the table and board.
  This ensured solutions were reliably safe and feasible.
<br/><br/>

![Gripper holding piece]({{ site.baseurl }}/assets/images/gripper-holding-piece.jpg)

## Trade-Offs
We originally planned to **not** use AR markers because we thought the simple geometry of tetris pieces would be easy to identify with vision, but we were spending too much time trying to get it to work.
In the end, we fell back on AR markers, which helped improve robustness.

Each of the 7 tetris piece types has a unique AR marker rasterized on it to ensure precision.
The contrast between the dark raster and the light wood turned out to be poor as the camera could not reliably detect the AR markers, so the AR markers had to be painted.

Because all pieces of the same type share an AR marker, our implementation does not allow multiple pieces of the same type to be in front of the camera simultaneously. Instead, it handles the pieces one by one so that there is no potential for conflict in AR detection.

<br/><br/><br/>
