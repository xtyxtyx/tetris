"""
env -- Module managing the state of the environment.
"""

from __future__ import division, generators, print_function
import numpy as np
import rospy
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from solver import TILE_TYPES, AR_TAG, rotate
from tf.transformations import quaternion_from_euler, euler_from_quaternion


DEFAULT_POSITION = np.zeros(3)
DEFAULT_ORIENTATION = np.array([0, 0, 0, 1])


def create_pose(frame_id, position=None, orientation=None):
    """ Convert a pose as arrays into a ROS-compatible timestamped pose data type. """
    position = position if position is not None else DEFAULT_POSITION
    orientation = orientation if orientation is not None else DEFAULT_ORIENTATION
    target = PoseStamped()
    target.header.frame_id = frame_id
    target_pos, target_orien = target.pose.position, target.pose.orientation
    target_pos.x, target_pos.y, target_pos.z = position
    target_orien.x, target_orien.y, target_orien.z, target_orien.w = orientation
    return target


def convert_pose(pose):
    position, orientation = pose.pose.position, pose.pose.orientation
    return (np.array([position.x, position.y, position.z]),
            np.array([orientation.x, orientation.y, orientation.z, orientation.w]))


def log_pose(msg, position, orientation=None):
    """ Logs the given pose. """
    if orientation is None:
        rospy.loginfo('{} (x={}, y={}, z={})'.format(msg, *position))
    else:
        template = '{} (x={}, y={}, z={}, o_x={}, o_y={}, o_z={}, o_w={})'
        rospy.loginfo(template.format(msg, *np.concatenate((position, orientation))))


def marker_frame(marker_id):
    return 'ar_marker_{}'.format(marker_id)


def add_transform_offset(trans, translation=None):
    if translation is None:
        translation = np.zeros(3)
    offset = create_pose(trans.child_frame_id, translation)
    return do_transform_pose(offset, trans)


class Environment:
    def __init__(self):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)

    def get_transform(self, source_frame, target_frame, timeout=3):
        frames = '"{}" w.r.t. "{}"'.format(target_frame, source_frame)
        if rospy.get_param('verbose'):
            rospy.loginfo('Acquiring transform: ' + frames)
        start = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - start < timeout:
            try:
                return self.buffer.lookup_transform(source_frame, target_frame,
                                                    rospy.Time())
            except TransformException:
                pass
        if rospy.get_param('verbose'):
            rospy.logwarn('Failed to find transform: ' + frames)


class PNPEnvironment(Environment):
    FRAME_MARKER_ID = 7
    ROTATIONS = np.array([
        [0, 1, 0, 0],
        [1/2**0.5, 1/2**0.5, 0, 0],
        [1, 0, 0, 0],
        [1/2**0.5, -1/2**0.5, 0, 0],
    ])
    UPWARDS = np.array([0, 0, 0, 1])
    DOWNWARDS = np.array([0, -1, 0, 0])
    JOINT_NAMES = ['e0', 'e1', 's0', 's1', 'w0', 'w1', 'w2']
    NEUTRAL_POSITIONS = {
        'left': np.array([0.1, 0.75, 0.1]),
        'right': np.array([0.1, -0.75, 0.1]),
    }

    SEARCH_HEIGHT = rospy.get_param('search_height')
    SEARCH_POSITIONS = np.array([
        # [0.471, 0.3, SEARCH_HEIGHT],  # Board bottom left
        [0.75, 0.3, SEARCH_HEIGHT],  # Board top left
        # [0.75, -0.15, SEARCH_HEIGHT],  # Board top right
        [0.45, -0.4, SEARCH_HEIGHT],  # Board bottom right
    ])
    TABLE_OBSTACLE_NAME = 'table'

    def __init__(self, table_height=np.nan, frame_id='base',
                 tool_frame_id='right_gripper'):
        Environment.__init__(self)
        self.table_height = table_height
        self.frame_id, self.tool_frame_id = frame_id, tool_frame_id
        self.table_placed = False

    def get_joint_names(self, side):
        return [side + '_' + name for name in self.JOINT_NAMES]

    def get_rel_transform(self, frame):
        return self.get_transform(self.frame_id, frame)

    def get_gripper_transform(self):
        return self.get_rel_transform(self.tool_frame_id)

    def find_tile_center(self, tile_type, trans):
        rot = trans.transform.rotation
        _, _, e_z = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        rot.x, rot.y, rot.z, rot.w = quaternion_from_euler(0, np.pi, -e_z)
        offset = add_transform_offset(trans, [tile_type.x_offset, tile_type.y_offset, 0])
        return convert_pose(offset)

    def place_table(self, planner, trans):
        # Offsets are in meters
        tile_size = rospy.get_param('tile_size')
        x_offset = (rospy.get_param('board_height') + 1)/2*tile_size/100
        y_offset = -(rospy.get_param('board_width') + 1)/2*tile_size/100
        z_offset = -2*rospy.get_param('board_thickness') - rospy.get_param('table_thickness')/2
        pose = add_transform_offset(trans, np.array([x_offset, y_offset, z_offset]))
        center_pos, center_orien = convert_pose(pose)
        dimensions = np.array([-4*x_offset, -4*y_offset, rospy.get_param('table_thickness')])
        planner.add_box_obstacle(dimensions, self.TABLE_OBSTACLE_NAME, center_pos, self.UPWARDS)
        self.table_placed = True
        if rospy.get_param('verbose'):
            log_pose('Placed table.', center_pos, center_orien)

    def find_ar_tag_position(self, tile):
        pattern = np.array(TILE_TYPES[tile.tile_name].pattern, dtype=np.int)
        rows, columns = pattern.shape
        for row in range(rows):
            for column in range(columns):
                if pattern[row, column] == AR_TAG:
                    return row, column
        raise ValueError('AR tag not found in pattern.')

    def find_slot_transform(self, tile, table_trans, z_pos):
        tile_size = rospy.get_param('tile_size')
        row, column = self.find_ar_tag_position(tile)
        rospy.loginfo('Relative AR tag position: ({}, {})'.format(row, column))

        row, column = row + tile.row, column + tile.column
        x_offset = (column + 1)*tile_size
        y_offset = -(row + 1)*tile_size
        # TODO: refactor
        if tile.rotations == 0:
            x_offset += tile_type.x_offset
            y_offset += tile_type.y_offset
        elif tile.rotations == 1:
            x_offset += tile_type.y_offset
            y_offset -= tile_type.x_offset
        elif tile.rotations == 2:
            x_offset -= tile_type.x_offset
            y_offset -= tile_type.y_offset
        else:
            x_offset -= tile_type.y_offset
            y_offset += tile_type.x_offset
        offset = add_transform_offset(table_trans, np.array([x_offset/100, y_offset/100, z_pos]))
        return convert_pose(offset)
