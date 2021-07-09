"""
planner -- Module for performing path planning.
"""

from __future__ import division, generators, print_function
import sys
from moveit_commander import (RobotCommander, PlanningSceneInterface, MoveGroupCommander,
                              roscpp_initialize)
from moveit_msgs.msg import OrientationConstraint, Constraints, CollisionObject
import rospy
from shape_msgs.msg import SolidPrimitive
from env import create_pose, log_pose

roscpp_initialize(sys.argv)


class PathPlanner:
    """
    This path planner wraps the planning and actuation functionality provided by MoveIt.

    All positions are arrays holding x, y, and z coordinates. Orientations are
    arrays holding x, y, z, and w coordinates (in quaternion form).

    Attributes:
        frame_id (str): The frame all coordinates are relative to.
        workspace (list): The bounds of the planning space (a box). Specified
            as the minimum x, y, and z coordinates, then the maximum x, y, and
            z coordinates.
        group_name (str): The MoveIt group name (for example, 'right_arm').
        time_limit (float): The maximum number of seconds MoveIt will plan for.
        robot: The MoveIt robot commander.
        scene: The planning scene.
        group: The MoveIt MoveGroup.
        scene_publisher: A publisher that updates the planning scene.
    """
    PLANNING_SCENE_TOPIC = '/collision_object'

    def __init__(self, frame_id, group_name, time_limit=10, workspace=None,
                 register_shutdown=True):
        if workspace is None:
            workspace = [-2, -2, -2, 2, 2, 2]
        if rospy.get_param('verbose'):
            rospy.loginfo('Move group: {}'.format(group_name))
        self.frame_id, self.workspace = frame_id, workspace
        self.time_limit, self.group_name = time_limit, group_name
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.scene_publisher = rospy.Publisher(self.PLANNING_SCENE_TOPIC,
                                               CollisionObject, queue_size=10)
        self.group = MoveGroupCommander(group_name)

        if register_shutdown:
            rospy.on_shutdown(self.shutdown)
        self.group.set_planning_time(time_limit)
        self.group.set_workspace(workspace)
        rospy.sleep(0.5)  # Sleep to ensure initialization has finished.
        if rospy.get_param('verbose'):
            rospy.loginfo('Initialized path planner.')

    def shutdown(self):
        """ Stop the path planner safely. """
        self.group.stop()
        del self.group
        if rospy.get_param('verbose'):
            rospy.logwarn('Terminated path planner.')

    def move_to_pose(self, position, orientation=None):
        """
        Move the end effector to the given pose naively.

        Arguments:
            position: The x, y, and z coordinates to move to.
            orientation: The orientation to take (quaternion, optional).

        Raises (rospy.ServiceException): Failed to execute movement.
        """
        if orientation is None:
            self.group.set_position_target(position)
        else:
            self.group.set_pose_target(create_pose(self.frame_id, position, orientation))
        if rospy.get_param('verbose'):
            log_pose('Moving to pose.', position, orientation)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def move_to_pose_with_planner(self, position, orientation=None, orientation_constraints=None):
        """
        Move the end effector to the given pose, taking into account
        constraints and planning scene obstacles.

        Arguments:
            position: The x, y, and z coordinates to move to.
            orientation: The orientation to take (quaternion, optional).
            orientation_constraints (list): A list of `OrientationConstraint`
                objects created with `make_orientation_constraint`.

        Returns (bool): Whether or not the movement executed successfully.
        """
        if orientation_constraints is None:
            orientation_constraints = []
        target = create_pose(self.frame_id, position, orientation)
        if rospy.get_param('verbose'):
            log_pose('Moving to pose with planner.', position, orientation)
        for _ in range(3):
            try:
                plan = self.plan_to_pose(target, orientation_constraints)
                if not self.group.execute(plan, True):
                    raise ValueError('Execution failed.')
            except ValueError as exc:
                rospy.logwarn('Failed to perform movement: {}. Retrying.'.format(str(exc)))
            else:
                break
        else:
            raise ValueError('Failed to move to pose.')

    def plan_to_pose(self, target, orientation_constraints):
        """
        Plan a movement to a pose from the current state, given constraints.

        Arguments:
            target: The destination pose.
            orientation_constraints (list): The constraints.

        Returns (moveit_msgs.msg.RobotTrajectory): The path.
        """
        self.group.set_pose_target(target)
        self.group.set_start_state_to_current_state()
        constraints = Constraints()
        constraints.orientation_constraints = orientation_constraints
        self.group.set_path_constraints(constraints)
        return self.group.plan()

    def add_box_obstacle(self, dimensions, name, com_position, com_orientation):
        """
        Add a rectangular prism obstacle to the planning scene.

        Arguments:
            dimensions: An array containing the width, length, and height of
                the box (in the box's body frame, corresponding to x, y, and z).
            name: A unique name for identifying this obstacle.
            com_position: The position of the center-of-mass (COM) of this box,
                relative to the global frame `frame_id`.
            com_orientation: The orientation of the COM.
        """
        pose = create_pose(self.frame_id, com_position, com_orientation)
        obj = CollisionObject()
        obj.id, obj.operation, obj.header = name, CollisionObject.ADD, pose.header
        box = SolidPrimitive()
        box.type, box.dimensions = SolidPrimitive.BOX, dimensions
        obj.primitives, obj.primitive_poses = [box], [pose.pose]
        self.scene_publisher.publish(obj)
        if rospy.get_param('verbose'):
            rospy.loginfo('Added box object "{}" to planning scene: '
                          '(x={}, y={}, z={}).'.format(name, *dimensions))

    def remove_obstacle(self, name):
        """ Remove a named obstacle from the planning scene. """
        obj = CollisionObject()
        obj.id, obj.operation = name, CollisionObject.REMOVE
        self.scene_publisher.publish(obj)
        if rospy.get_param('verbose'):
            rospy.loginfo('Removed object "{}" from planning scene.'.format(name))

    def make_orientation_constraint(self, orientation, link_id, tolerance=0.1, weight=1):
        """
        Make an orientation constraint in the context of the robot and world.

        Arguments:
            orientation: The orientation the link should have.
            link_id: The name of the link frame.

        Returns (OrientationConstraint): The constraint.
        """
        constraint = OrientationConstraint()
        constraint.header.frame_id = self.frame_id
        constraint.link_name = link_id
        const_orien = constraint.orientation
        const_orien.x, const_orien.y, const_orien.z, const_orien.w = orientation
        constraint.absolute_x_axis_tolerance = tolerance
        constraint.absolute_y_axis_tolerance = tolerance
        constraint.absolute_z_axis_tolerance = tolerance
        constraint.weight = weight
        return constraint
