### -----------------------------------------------------------
# new_th = unwrap(th,prev_th) unwraps the radian phase angle th.
# Whenever the jump between consecutive angles is greater than or equal
# to pi radians, unwrap shifts the angles by adding multiples of +/-2pi
# until the jump is less than pi.
#
# Inspired by Matlab's unwrap function:
# https://www.mathworks.com/help/matlab/ref/unwrap.html
### -----------------------------------------------------------

import math

def unwrap(th, prev_th):

	new_th = th
	#print(new_th, "0")
	while (new_th - prev_th) >= math.pi:
		new_th -= 2*math.pi
		#print(new_th, "1")
	while (new_th - prev_th) <= -math.pi:
		new_th += 2*math.pi
		#print(new_th, "2")

	return new_th
