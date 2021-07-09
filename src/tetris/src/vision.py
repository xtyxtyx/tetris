#!/usr/bin/env python

from __future__ import division, generators, print_function, unicode_literals

import matplotlib.pyplot as plt
import numpy as np
import rospy
from scipy import signal, ndimage
from sensor_msgs.msg import Image

LEFT_CAM = '/cameras/left_hand_camera/image'
RIGHT_CAM = '/cameras/right_hand_camera/image'

GAUSSIAN_KERNEL = np.outer(signal.gaussian(10, 2), signal.gaussian(10, 2))

SUCTION_CAM = RIGHT_CAM  # FIXME: make a command-line option

SHOWN = False  # HACK

def rgb_to_grayscale(r, g, b):
	""" Approximate an RGB image as grayscale. """
	return 0.3*r + 0.59*g + 0.11*b

def suction_cam_callback(image):
	assert image.encoding == 'bgra8'
	raw = np.fromstring(image.data, dtype=np.uint8).reshape((image.height, image.width, 4))
	colorized = np.stack((raw[:,:,2], raw[:,:,1], raw[:,:,0]), axis=2)
	grayscale = rgb_to_grayscale(raw[:,:,2], raw[:,:,1], raw[:,:,0]).astype(np.uint8)
	blurred = signal.fftconvolve(grayscale, GAUSSIAN_KERNEL, mode='same')
	edges = ndimage.sobel(blurred)

	dev_mean = np.mean(edges)
	dev_bitmap = 1 - (np.abs(edges - dev_mean) > 10*dev_mean).astype(np.int)

	gray_mean = np.mean(grayscale)
	gray_bitmap = ((grayscale - gray_mean) > 3*gray_mean).astype(np.int)

	combined_bitmap = dev_bitmap/dev_bitmap.max() + 2*gray_bitmap/gray_bitmap.max()

	# saturated = np.minimum(0.97*256, np.maximum(0.03*256, blurred)) # - 0.05*256)/0.9

	# b, a = signal.butter(2, 0.001, 'highpass')
	# deglared = signal.lfilter(b, a, grayscale)
	# deglared = grayscale

	# processed = blurred

	global SHOWN
	if not SHOWN:
		SHOWN = True
		# plt.imshow(colorized)
		plt.imshow(combined_bitmap, cmap='gray')
		plt.show()

def main():
	rospy.init_node('vision')
	rospy.Subscriber(SUCTION_CAM, Image, suction_cam_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
