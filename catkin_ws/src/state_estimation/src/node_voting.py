#!/usr/bin/env python

import rospy
import numpy as np
from state_estimation.msg import AlgDecision


class VotingDecisionNode:

	def __init__(self):

		rospy.init_node('node_voting', anonymous=True)

		self.decision_publisher = rospy.Publisher('state_estimation/voting_decision', AlgDecision, queue_size=1)

		self.exnis_sub = rospy.Subscriber('state_estimation/exnis_decision', AlgDecision, lambda data: self.decision_callback(data,0))
		self.tree_sub = rospy.Subscriber('state_estimation/tree_decision', AlgDecision, lambda data: self.decision_callback(data,1))
		self.knn_sub = rospy.Subscriber('state_estimation/knn_decision', AlgDecision, lambda data: self.decision_callback(data,2))
		self.ffnn_sub = rospy.Subscriber('state_estimation/ffnn_decision', AlgDecision, lambda data: self.decision_callback(data,3))
		self.rnn_sub = rospy.Subscriber('state_estimation/rnn_decision', AlgDecision, lambda data: self.decision_callback(data,4))

		self.alg_decision = AlgDecision()

		n_alg = 5
		n_sources = 4
		self.decisions = np.ones((n_alg,n_sources), dtype = bool)
		self.available = np.zeros(n_alg, dtype = bool)


	def decision_callback(self, data, alg):


		self.decisions[alg,:] = data.selected
		self.available[alg] = True

		self.vote()


	def vote(self):

		## Voting system (majority)

		if self.available.all():

			### Compute the majority vote (mode)
			# Numpy doesn't have a mode function, but knowing that labels are either 0 or 1
			# we can compute the mode by just averaging and thresholding by 0.5
			final_decision = np.mean(self.decisions.astype(float), axis=0)
			final_decision = final_decision > 0.5

			# Last-resort source: Cartographer
			if not final_decision.any():
				final_decision = np.array([True, False, False, False])

			### Publish topic with algorithm decision
			self.alg_decision.stamp = rospy.Time.now()
			self.alg_decision.selected = final_decision
			self.decision_publisher.publish(self.alg_decision)

			### Reset availability flags
			self.available.fill(False)

		else:
			pass


	def main_loop(self):
		while not rospy.is_shutdown():
			rospy.spin()




if __name__ == '__main__':
	state_estimation_node = VotingDecisionNode()

	try:
		state_estimation_node.main_loop()
	except rospy.ROSInterruptException:
		pass
