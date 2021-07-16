#!/usr/bin/env python

import rospy
import numpy as np
from state_estimation.msg import ProcessedData, AlgDecision


class ExnisDecisionNode:

	def __init__(self):

		rospy.init_node('node_exnis', anonymous=True)

		self.decision_publisher = rospy.Publisher('state_estimation/exnis_decision', AlgDecision, queue_size=1)

		self.data_sub = rospy.Subscriber('state_estimation/processed_data', ProcessedData, self.data_callback)

		self.alg_decision = AlgDecision()

		self.v1j = np.zeros((1,3))
		self.v2j = np.zeros((1,3))
		self.v3j = np.zeros((1,3))
		self.v4j = np.zeros((1,3))
		self.k_corr = 1

		self.th1 = 6.25 # 6.25 = chi-square 3-dof 10% prob. of exceeding that value
		self.th2 = 7.82 # 7.82 = chi-square 3-dof 5% prob. of exceeding that value
		self.beta = 0.5 # 0.5 = approx. 2 time steps average (higher weights to recent values)


	def data_callback(self, data):

		## ExNIS cross-validation

		### Collect dij from message data
		d1j = np.array([data.dij[0], data.dij[1], data.dij[2]])
		d2j = np.array([data.dij[0], data.dij[3], data.dij[4]])
		d3j = np.array([data.dij[1], data.dij[3], data.dij[5]])
		d4j = np.array([data.dij[2], data.dij[4], data.dij[5]])

		### EWA
		# Filtering
		self.v1j = self.beta * self.v1j + (1 - self.beta) * d1j
		self.v2j = self.beta * self.v2j + (1 - self.beta) * d2j
		self.v3j = self.beta * self.v3j + (1 - self.beta) * d3j
		self.v4j = self.beta * self.v4j + (1 - self.beta) * d4j
		# Bias correction
		if self.k_corr < 700: # until it has no significant effect
			c1j = self.v1j / (1 - self.beta ** self.k_corr)
			c2j = self.v2j / (1 - self.beta ** self.k_corr)
			c3j = self.v3j / (1 - self.beta ** self.k_corr)
			c4j = self.v4j / (1 - self.beta ** self.k_corr)
			self.k_corr += 1
		else:
			c1j = self.v1j
			c2j = self.v2j
			c3j = self.v3j
			c4j = self.v4j

		### 2-lvl combined thresholding
		selected1 = np.array([np.sum(c1j <= self.th1) >= 1, np.sum(c2j <= self.th1) >= 1, \
					np.sum(c3j <= self.th1) >= 1, np.sum(c4j <= self.th1) >= 1])
		selected2 = np.array([np.sum(c1j <= self.th2) >= 2, np.sum(c2j <= self.th2) >= 2, \
					np.sum(c3j <= self.th2) >= 2, np.sum(c4j <= self.th2) >= 2])
		selected = np.logical_or(selected1, selected2)

		if not selected.any(): # in case rule discards everyone
			selected = np.array([True, False, False, False]) # rely on Cartographer

		### Publish topic with algorithm decision
		self.alg_decision.stamp = rospy.Time.now()
		self.alg_decision.selected = selected
		self.decision_publisher.publish(self.alg_decision)


	def main_loop(self):
		while not rospy.is_shutdown():
			rospy.spin()




if __name__ == '__main__':
	state_estimation_node = ExnisDecisionNode()

	try:
		state_estimation_node.main_loop()
	except rospy.ROSInterruptException:
		pass
