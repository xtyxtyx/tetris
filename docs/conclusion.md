---
title: Conclusion
about: conclusion
---
<!--(a) Discuss your results. How well did your finished solution meet your design criteria? 2 (b) Did you encounter any particular difficulties? (c) Does your solution have any flaws or hacks? What improvements would you make if you had additional time?-->

## Fulfillment of criteria
* Able to find the correct pose of pieces from their AR marker transform
* Able to pick up each piece in a stable manner by grasping at its red dot with the vacuum gripper
* Able to detect the board with AR markers and therefore we can avoid knocking into the board and the table underneath it by adding a box obstacle to the `MoveIt` planner
* Able to compute each piece's solution place using a tetris solver algorithm and `tf2` transforms
* Able to place pieces in their solution pose using `MoveIt`
* Not yet able to have pieces fit flush on board

<br>
## Challenges
* Unstable AR marker estimates.
  The AR markers are rasterized onto the wooden pieces via laser cutter.
  The raster did not produce good enough contrast between the light and dark parts of the marker, so we had to paint the dark parts black.
  This allowed AR detection to work in most cases, however, good lighting is still a non-negligible factor.

   We found that AR detection would sometimes fit incorrect axes onto the pieces.
   Since the camera is peering down on the scene, the axes' rotation about the Z-axis (going up out of the table) was usually very accurate.
   However, the orientation about the X and Y would sometimes jump around.
   For our task, we only care about the orientation about the Z anyway since we can assume the pieces are flat on the table.
   Therefore, in our code we convert the orientation quaternion from the AR marker transform into Euler angles, zero any rotation about the X and Y, convert back to quaternion and use that as the true AR marker transform.

* Tile types share an AR marker ID.
  AR detection will have conflicts if there are multiple markers with the same exact ID in the camera view at the same time. Therefore, we choose to feed the robot pieces one-by-one.
  We have the solver output a solution for all the pieces. Then proceed in row major order to pick-and-place the pieces.
  This is the order we feed the pieces to the robot. When the robot is ready to pick-and-place the next piece, the code will ask for a piece of appropriate type via terminal output.

* Bad grasps (depressions on AR marker).
  The vacuum gripper is not able to get good suction on the parts of the pieces overlapping with the rasterized AR marker. This is because the raster creates an uneven surface. Therefore we have to offset where we decided to pick up the pieces (the red dot).

* We thought we had to start the joint state publisher in order to properly receive AR marker transforms from the left hand camera.
  It turns out that when we did that, we were not passing in the right parameters and the published joint states were nonsense, leading to nonsense AR marker positions.
  This took us a significant amount of debugging.
  In the end, we restarted the Baxter robot and, using only the default robot state publisher, were able to get accurate AR marker readings.

* Hardware requires hacky fixes.
  The seal on the vacuum pipe got deformed as people were constantly changing grippers on the Baxter machines.
  As a result, the vacuum was no longer creating a tight seal and we could not grasp pieces reliably with the suction gripper.
  We ended up wrapping tape around the pipe and twisting tape into the seal to make it work.

<br>
## Improvements
* Robust AR tracking.
  We can raster two AR markers per piece for greater pose stability and more robust initial detection.
  We used the bundle feature of `ar_track_alvar` to create very reliable poses for the board, so bundling tags on tiles would be a valuable extension.

* Placing tags away from the tile center of mass.
  This will allow us to move the red dot of each piece (grasp point) to its true center of mass.
  This, in turn, improves the stability of the gripper while the attached arm moves, especially if the robot is moving along an unpredictable path.

* Assigning unique AR tags to each piece.
  This would enable us to have all the pieces laid out on the board at once, while still allowing AR detection to work.
  This eliminates the need for a human to standby and feed the pieces one by one.
  The system would be fully autonomous.

* Use vision without AR markers.
  This would extend the applicability of our project to pieces without AR markers (i.e. most things in the world).

<br/><br/><br/>
