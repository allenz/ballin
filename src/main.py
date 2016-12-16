#!/usr/bin/env python
# Integrated code for sensing, planning, and moving
# For Baxter or a standalone robot
# Written by Allen Zhu
# BSD License

import rospy
import tf
import numpy as np
from calcs import calcTilt

######################### Immutable Config #########################
BAXTER = False # True if using Baxter
AR_TAG = 'ar_marker_11' # Marker name, check via RViz
A_PORT = '/dev/ttyACM2' # Standalone only: serial port, check ls /dev/*
MUZZLE_V = 3.4 # Muzzle velocity
# camera coords: x horizontal, y vertical, z out
CAM_OFFSET = np.array([0,0.02,0]) # assumes parallel to launcher
AR_OFF_d = 0.07 # AR Tag offsets
AR_OFF_h = 0
A_STEP = 200.0/(2*np.pi) # convert degrees -> motor steps
# Adjust tilt angle to account for launch mechanism wobble
tiltAdj = lambda d: np.interp(d, [1.32, 1.04, 0.82], [0.02, -0.03, -0.06])
# Adjust pan angle to account for slanted tilt axis
panAdj = lambda d: np.interp(d, [1.04, 0.85], [0, 0.06])
####################################################################
if BAXTER:
	import baxter_interface
else:
	import serial

class g: # Global state
	listener = None
	d = None
	h = None
	a_ser = None # serial connection to Arduino
	b_limb = None
	b_rate = None

def pan():
	try:
		for _ in range(6):
			t, r = g.listener.lookupTransform('usb_cam', AR_TAG, rospy.Time(0))
			trans = np.array(t) - CAM_OFFSET
			angle = np.arctan2(trans[0], trans[2]) + panAdj(g.d)
			if(round(A_STEP*angle) == 0.0):
				return
			print('Panning', -angle)
			move('right_s0', -angle)
	except Exception, e:
		print(e)

def measureDH():
	"""Measure the distance and height of the target."""
	try:
		t, r = g.listener.lookupTransform('usb_cam', AR_TAG, rospy.Time(0))
		trans = np.array(t) - CAM_OFFSET
		g.d, g.h = trans[2] + AR_OFF_d, -trans[1] - AR_OFF_h
		print('d, h', g.d, g.h)
	except Exception, e:
		print(e)

def tilt():
	"""Tilt to face AR tag."""
	try:
		angle = calcTilt(MUZZLE_V, g.d, g.h) + tiltAdj(g.d)
		if BAXTER:
			tilt_goal = angle + g.b_limb.joint_angle('right_e1')
		print('Tilting', angle)
		move('right_e1', angle)
		if BAXTER:
			print('Tilting', g.b_limb.joint_angle('right_e1')-tilt_goal)
			move('right_e1', g.b_limb.joint_angle('right_e1')-tilt_goal)
			print('Tilting', g.b_limb.joint_angle('right_e1')-tilt_goal)
			move('right_e1', g.b_limb.joint_angle('right_e1')-tilt_goal)
	except Exception, e:
		print(e)

def findHoop():
	"""Arduino: check all alround"""
	if not BAXTER:
		for _ in range(6):
			g.listener.waitForTransform('usb_cam', AR_TAG, rospy.Time(), rospy.Duration(0.5))
			move('right_s0', 60)

def moveInitial():
	if BAXTER:
		# right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2
		g.b_limb = baxter_interface.Limb('right')
		g.b_rate = rospy.Rate(2) # 2 hz
		init_pos = {}
		for jt, pos in zip(g.b_limb.joint_names(),
						   [np.pi/4, 0, 0, np.pi/2, 0, 0, 0]):
			init_pos[jt] = pos

		print('Moving to initial position')
		for _ in range(6):
			g.b_limb.set_joint_positions(init_pos) # blocking method?
			g.b_rate.sleep()
	else:
		# move('right_e1', np.pi/2) # set tilt to horizontal
		pass

	# Initialize transforms
	g.listener = tf.TransformListener()
	g.listener.waitForTransform('usb_cam', AR_TAG, rospy.Time(), rospy.Duration(4))

def move(joint, angle):
	"""Baxter: move joint by angle."""
	if BAXTER:
		joint_command = {joint: angle + g.b_limb.joint_angle(joint)}
		for _ in range(20):
			g.b_limb.set_joint_positions(joint_command)
			g.b_rate.sleep()
	else:
		steps = round(A_STEP*angle)
		print(steps)
		g.a_ser.write(str(int((2000 if joint == 'right_e1' else 0) + steps)) + ' ')
		rospy.sleep(round(np.abs(.25*steps))+1.5)

def setup():
	rospy.init_node("ballin")
	if BAXTER: # Enable robot and restore enabled state after exit
		rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
		init_state = rs.state().enabled
		def clean_shutdown():
			if not init_state:
				print("Disabling Baxter")
				rs.disable()
		rospy.on_shutdown(clean_shutdown)
		rs.enable()
	else: # Open serial connection
		g.a_ser = serial.Serial(A_PORT, 9600)
		rospy.sleep(2)

def main():
	setup()
	moveInitial()
	# findHoop()
	measureDH()
	pan()
	measureDH()
	tilt()

if __name__=='__main__':
	main()
