#!/usr/bin/env python2

"""
References:
  * https://github.com/RethinkRobotics/baxter_interface/blob/release-0.7.0/src/baxter_interface/camera.py
"""

from __future__ import division, generators, print_function

import click
from baxter_interface.camera import CameraController


@click.command()
@click.option('-c', '--camera-name', required=True, help='Camera name')
def main(camera_name):
    camera = CameraController(camera_name)
    print('Resolution:', camera.resolution)
    print('Frames per second:', camera.fps)
    print('Exposure:', camera.exposure)
    print('Gain:', camera.gain)
    print('White balance:')
    print('  Red:', camera.white_balance_red)
    print('  Green:', camera.white_balance_green)
    print('  Blue:', camera.white_balance_blue)
    print('Flip:', camera.flip)
    print('Mirror:', camera.mirror)
    print('Binning/half resolution:', camera.half_resolution)


if __name__ == '__main__':
    main()
