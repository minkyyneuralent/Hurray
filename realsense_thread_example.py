#!/usr/bin/env python3

import argparse
import signal 
from datetime import datetime 
from dateutil import tz
import time
import os 
import struct 
import threading
import numpy as np 

import sys
import cv2
import pyrealsense2 as rs

FLAGS = None
main_running = True


########################################################################################
class D435Saver(threading.Thread):
	def __init__(self, device_id=None, sampling_rate=10):
		threading.Thread.__init__(self)
		self.device_id = device_id 
		self.sampling_rate = sampling_rate        

		self.mutex = threading.Lock()

		self.depth = None 
		self.aligned_depth = None
		self.timestamp = 0.0 
		
		self.pipeline_2 = rs.pipeline()
		self.config_2 = rs.config()
		self.config_2.enable_device(self.device_id) if self.device_id is not None else 0
		self.config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		self.pipeline_2.start(self.config_2)

	def get_depth_data(self):
		return self.depth

	def get_aligned_depth_data(self):
		return self.aligned_depth

	def run(self):
		global main_running

		while main_running:
			frames = self.pipeline_2.wait_for_frames()
			if frames:
				rs_color_frame = frames.get_color_frame()
				rs_depth_frame = frames.get_depth_frame()

				align = rs.align(rs.stream.color)
				processed = align.process(frames)
				rs_aligned_depth_frame = processed.get_depth_frame()

				self.mutex.acquire()
				self.depth = np.asanyarray(rs_depth_frame.get_data()) if rs_depth_frame else None
				self.aligned_depth = np.asanyarray(rs_aligned_depth_frame) if rs_aligned_depth_frame else None 
				self.mutex.release()
			time.sleep(1.0 / (self.sampling_rate * 2))
		self.pipeline_2.stop()


########################################################################################
def sigHandler(_signo, _stack_frame):
	global main_running
	main_running = False 


##################################################################################################################################
def main():
	try:
		global main_running

		signal.signal(signal.SIGTERM, sigHandler)
		signal.signal(signal.SIGABRT, sigHandler)
		signal.signal(signal.SIGINT, sigHandler)

		# D435
		d435_saver = D435Saver(device_id=FLAGS.device_id, sampling_rate=15)
		if d435_saver is not None:
			d435_saver.daemon = True
			d435_saver.start()
		print("D435 Saver Start...")

		depth_image = d435_saver.get_depth_data()
		aligned_depth_image = d435_saver.get_aligned_depth_data()

		### ~~~~~
		### ~~~~~


		print("Close.")

	except Exception as ex:
		print("Exception (Line {}) : {}".format(sys.exc_info()[-1].tb_lineno, ex))


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("--device_id", type=str, default=None, help="D435 Device ID")

	FLAGS, _ = parser.parse_known_args()

	main()

