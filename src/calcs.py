#!/usr/bin/env python
# Compute the tilt angle to reach a target position
# Written by Allen Zhu
# BSD License

import numpy as np
from scipy.optimize import fsolve

g = 9.8 # gravity

def calcTilt(v, d, h):
	"""Returns tilt angle to hit target at dist d and height h with muzzle v."""

	# Impossibility condition: at theta = pi/4 = 0.785, is our height at distance d > h?
	# Not fully accurate when h != 0
	if d - g*(d/v)**2 < h:
		print('Impossible to reach hoop.')

	# Height error function
	def e(theta):
		if theta < 0 or theta > np.pi/2: # np.pi/4
			return 1e99
		return d*np.tan(theta) - g/2*(d/(v*np.cos(theta)))**2 - h

	# Numerical solution
	theta = fsolve(e, np.pi/2-1e-5)[0]
	
	if e(theta) > 0.1:
		print('Unable to find a working tilt angle.')
	return theta
