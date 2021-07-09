#!/usr/bin/env python

"""
tetris -- Solves a Tetris-like puzzle.
"""

from __future__ import division, generators, print_function
import os
import traceback
import cv2
import cv_bridge
import rospy
import rospkg
from sensor_msgs.msg import Image
from pnp import TetrisPNPTask
from solver import TILE_TYPES, solve_puzzle, optimize_solution, display_solution


def send_image(img_pub, path, delay=1):
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
    img_pub.publish(msg)
    rospy.sleep(delay)


def get_board_state():
    """
    Returns (int, int, dict): Rows, columns, and tile distribution.
    """
    tiles = {name: rospy.get_param('board_{}_tiles'.format(name)) for name in TILE_TYPES}
    return rospy.get_param('board_height'), rospy.get_param('board_width'), tiles


def solve_puzzle_optimized():
    rows, columns, tiles = get_board_state()
    solution = solve_puzzle(rows, columns, tiles)
    assert solution is not None, 'Failed to solve puzzle.'
    solution = optimize_solution(solution)
    if rospy.get_param('verbose'):
        print('\n')
        display_solution(rows, columns, solution, scale=4, offset=4)
        print('\n')
        for tile in solution:
            rospy.loginfo('Tile: ({}, {}), type={}, rotations={}'.format(tile.row, tile.column, tile.tile_name, tile.rotations))
    return solution


def main():
    rospy.init_node('tetris')
    # Wait for other nodes to come online.
    rospy.sleep(rospy.get_param('init_delay'))
    rospack = rospkg.RosPack()
    solution, i = solve_puzzle_optimized(), 0
    task = TetrisPNPTask()
    img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=5)

    while not rospy.is_shutdown() and i < len(solution):
        tile = solution[i]
        prompt = '(Please provide a {} tile. Press enter when done.) '

        path = os.path.join(rospack.get_path('tetris'),
                                'data/{}.png'.format(tile.tile_name))
        send_image(img_pub, path)
        raw_input(prompt.format(tile.tile_name.upper()))
        try:
            board_trans, tile_trans = task.pick(tile.tile_name)
            task.place(tile, board_trans, tile_trans)
        except Exception as exc:
            # Retry the current piece in the event of an error
            rospy.logerr(type(exc).__name__ + ': ' + str(exc))
            traceback.print_exc()
            task.open_gripper()
            continue
        else:
            i += 1


if __name__ == '__main__':
    main()
