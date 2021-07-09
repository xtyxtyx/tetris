"""
pnp -- Module for performing pick-and-place tasks.
"""

from __future__ import division, generators, print_function
import numpy as np
from baxter_interface import Gripper, AnalogIO, Limb
import rospy
from env import log_pose, marker_frame, add_transform_offset, convert_pose, PNPEnvironment
from planner import PathPlanner
from solver import TILE_TYPES, rotate, AR_TAG
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SuctionPNPTask:
    """
    A representation of a generic pick-and-place task using a suction cup.

    Attributes:
        gripper_side (str): 'left' or 'right'.
        planner: The path planner.
        gripper: The gripper.
        vacuum_sensor: The vacuum sensor.
    """
    GRIP_MAX_VALUE = 175

    def __init__(self, frame_id='base', gripper_side='right'):
        assert gripper_side in ('left', 'right')
        self.gripper_side = gripper_side
        self.camera_side = 'left' if self.gripper_side == 'right' else 'right'
        self.gripper_planner = PathPlanner(frame_id, self.gripper_side + '_arm')
        self.camera_planner = PathPlanner(frame_id, self.camera_side + '_arm')
        self.calibrate_gripper()
        if rospy.get_param('verbose'):
            rospy.loginfo('Initialized PNP task.')

    def set_joint_angles(self, angles, limb, tolerance=0.05, max_steps=1000, period=0.01):
        all_angles = dict(angles)
        for name in limb.joint_names():
            if name not in all_angles:
                all_angles[name] = limb.join_angle(name)

        step = 0
        done = lambda: all(abs(angle - limb.joint_angle(name)) < tolerance
                           for name, angle in all_angles.items())
        while step < max_steps and not done():
            limb.set_joint_positions(all_angles)
            rospy.sleep(period)
            step += 1
        return done()

    def is_grasping(self, threshold=80):
        """
        Uses the vacuum sensor to detect whether the suction cup has grasped an
        object. The topics

            /robot/analog_io/left_vacuum_sensor_analog/value_uint32
            /robot/analog_io/left_vacuum_sensor_analog/state

        both give the direct analog readings from the vacuum sensor in the
        gripper. According to our gripper engineer, the values mean:

            0 - 46:   The vacuum gripper is likely not attached to an object.
            47 - 175: The vacuum is likely attached to an object (usually
                      around 150 when grasping).
            176+:     There is likely a short between 5V and signal on the sensor.

        Arguments:
            threshold (int): Values above this threshold constitute a grip.
        """
        gripper_value = self.vacuum_sensor.state()
        if rospy.get_param('verbose'):
            rospy.loginfo('Current vacuum value: {}'.format(gripper_value))
        if gripper_value > self.GRIP_MAX_VALUE:
            raise ValueError('Detected unsafe vacuum value of {}.'.format(gripper_value))
        return gripper_value > threshold

    def calibrate_gripper(self):
        """ Initialize the gripper and its vacuum sensor. """
        self.gripper = Gripper(self.gripper_side)
        self.gripper.calibrate()
        self.gripper.open()
        self.vacuum_sensor = AnalogIO(self.gripper_side + '_vacuum_sensor_analog')
        if rospy.get_param('verbose'):
            rospy.loginfo('Calibrated gripper. (type={})'.format(self.gripper.type()))

    def open_gripper(self, delay=1):
        """ Open the gripper with a given delay afterwards. """
        self.gripper.open()
        rospy.sleep(delay)
        if rospy.get_param('verbose'):
            rospy.loginfo('Opened gripper.')

    def close_gripper(self, delay=1):
        """ Close a gripper with a given delay afterwards. """
        self.gripper.close()
        rospy.sleep(delay)
        if rospy.get_param('verbose'):
            rospy.loginfo('Closed gripper.')


class TetrisPNPTask(SuctionPNPTask):
    """
    A representation of the Tetris pick-and-place task.
    """
    def __init__(self, frame_id='base', gripper_side='right'):
        SuctionPNPTask.__init__(self, frame_id, gripper_side)
        self.env = PNPEnvironment(frame_id=frame_id)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.open_gripper()
        if self.env.table_placed:
            self.gripper_planner.remove_obstacle(self.env.TABLE_OBSTACLE_NAME)

    def search(self, frames, delay=2):
        # TODO: try to get multiple transforms to get a better estimate
        self.gripper_planner.move_to_pose_with_planner(
            self.env.NEUTRAL_POSITIONS[self.gripper_side], self.env.DOWNWARDS)
        frame_transforms, missing_frames = {}, []
        for position in self.env.SEARCH_POSITIONS:
            self.camera_planner.move_to_pose_with_planner(position, self.env.DOWNWARDS)
            rospy.sleep(delay)  # Allow frames to stabilize.
            for frame in frames:
                if frame not in frame_transforms:
                    trans = self.env.get_rel_transform(frame)
                    if trans is not None:
                        frame_transforms[frame] = trans
            if all(frame in frame_transforms for frame in frames):
                break
        else:
            for frame in frames:
                if frame not in frame_transforms:
                    missing_frames.append(frame)
            if rospy.get_param('verbose'):
                rospy.logwarn('Unable to find some frames: ' + ', '.join(missing_frames))

        self.camera_planner.move_to_pose_with_planner(
            self.env.NEUTRAL_POSITIONS[self.camera_side], self.env.DOWNWARDS)
        return frame_transforms, missing_frames

    def grasp(self, position, orientation):
        z_offset, z_delta = rospy.get_param('z_offset'), rospy.get_param('z_delta')
        z_max_steps = rospy.get_param('z_max_steps')

        current_pos = np.copy(position)
        current_pos[2] += z_offset
        self.gripper_planner.move_to_pose_with_planner(current_pos, orientation)
        if not rospy.get_param('grasp_enabled'):
            return

        steps = 0
        while not rospy.is_shutdown() and steps < z_max_steps:
            self.close_gripper()
            if not self.is_grasping():
                self.open_gripper()
                current_pos[2] -= z_delta
                self.gripper_planner.move_to_pose_with_planner(current_pos, orientation)
            else:
                self.env.table_height = current_pos[2] - rospy.get_param('board_thickness')
                log_pose('Grasped object.', position)
                break
            steps += 1
        else:
            if rospy.get_param('verbose'):
                log_pose('Exceeded number of steps during grasp.', position, orientation)

    def pick(self, tile_name):
        assert not self.is_grasping()
        board_id = marker_frame(rospy.get_param('board_top_left_marker'))
        tile_type = TILE_TYPES[tile_name]
        tile_id = marker_frame(tile_type.marker_id)
        frame_transforms, missing_frames = self.search([board_id, tile_id])
        if not self.env.table_placed and board_id in frame_transforms:
            self.env.place_table(self.gripper_planner, frame_transforms[board_id])
        if missing_frames:
            raise ValueError('Unable to find all transforms. Retry.')
        if rospy.get_param('verbose'):
            for name, trans in frame_transforms.items():
                position, orientation = convert_pose(add_transform_offset(trans))
                log_pose('Frame "{}"'.format(name), position, orientation)

        center_pos, center_orien = self.env.find_tile_center(
            tile_type, frame_transforms[tile_id])
        self.grasp(center_pos, center_orien)
        return frame_transforms[board_id], frame_transforms[tile_id]

    def rotate_to(self, orientation):
        trans = self.env.get_gripper_transform()
        translation = trans.transform.translation
        position = np.array([translation.x, translation.y, translation.z])
        self.gripper_planner.move_to_pose_with_planner(position, orientation)

    def place(self, tile, board_trans, tile_trans):
        if rospy.get_param('grasp_enabled'):
            assert self.is_grasping()
        lift = rospy.get_param('lift_offset')
        thickness = rospy.get_param('board_thickness')
        position, orientation = convert_pose(add_transform_offset(tile_trans))
        position[2] += lift + 2*thickness
        self.gripper_planner.move_to_pose_with_planner(position, orientation)

        _, _, e_z = euler_from_quaternion(convert_pose(add_transform_offset(board_trans))[1])
        orientation = quaternion_from_euler(0, np.pi, -(e_z - tile.rotations*np.pi/2))
        rospy.loginfo('Euler angle: ' + str(e_z))

        tile_size = rospy.get_param('tile_size')
        tile_type = TILE_TYPES[tile.tile_name]

        x, y = -tile_type.x_offset, -tile_type.y_offset
        if tile.rotations == 1:
            x, y = y, -x
        elif tile.rotations == 2:
            x, y = -x, -y
        elif tile.rotations == 3:
            x, y = -y, x
        grasp_to_ar_tag = np.array([x, y, 0])
        rospy.loginfo('Grasp to AR tag: ' + str(grasp_to_ar_tag))

        rotated_pattern = rotate(tile_type.pattern, tile.rotations)
        for ar_row in range(rotated_pattern.shape[0]):
            quit = False
            for ar_col in range(rotated_pattern.shape[1]):
                if rotated_pattern[ar_row, ar_col] == AR_TAG:
                    quit = True
                    break
            if quit:
                break
        rospy.loginfo('AR tag relative to top left: {}, {}'.format(ar_row, ar_col))
        ar_offset = np.array([(1 + tile.column + ar_col)*tile_size, -(1 + tile.row + ar_row)*tile_size, 0])
        rospy.loginfo('AR offset: ' + str(ar_offset))
        offset = (ar_offset/100 - grasp_to_ar_tag)
        rospy.loginfo('Offset: ' + str(offset))

        target, _ = convert_pose(add_transform_offset(board_trans, offset))
        target[2] = position[2]
        rospy.loginfo('Target: ' + str(np.round(target, 3)))
        self.gripper_planner.move_to_pose_with_planner(target, orientation)

        drop = np.copy(target)
        drop[2] = board_trans.transform.translation.z + 0.05
        self.gripper_planner.move_to_pose(drop, orientation)
        self.open_gripper()
        self.gripper_planner.move_to_pose_with_planner(target, orientation)

        self.gripper_planner.move_to_pose_with_planner(
            self.env.NEUTRAL_POSITIONS[self.gripper_side], self.env.DOWNWARDS)
