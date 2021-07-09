#!/usr/bin/env python

from __future__ import division, generators, print_function
import rospy
from pnp import SuctionPNPTask


def main():
    rospy.init_node('gripper_test')
    task = SuctionPNPTask()
    while not rospy.is_shutdown():
        raw_input('Press enter to close gripper. ')
        task.close_gripper()
        rospy.loginfo('Sensor reading: {}'.format(task.vacuum_sensor.state()))
        raw_input('Press enter to open gripper. ')
        task.open_gripper()


if __name__ == '__main__':
    main()
