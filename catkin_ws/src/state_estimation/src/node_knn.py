#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import os
import sklearn, joblib
from state_estimation.msg import ProcessedData, AlgDecision


class KNNDecisionNode:

	def __init__(self):

		rospy.init_node('node_knn', anonymous=True)

		self.knn_model = []
		r = rospkg.RosPack()
		for source in ['CG','GPS','DF','DR']:
			path = os.path.join(r.get_path('state_estimation'), "models/knn_" + source + '.sav')
			self.knn_model.append(joblib.load(path))

		self.decision_publisher = rospy.Publisher('state_estimation/knn_decision', AlgDecision, queue_size=1)

		self.data_sub = rospy.Subscriber('state_estimation/processed_data', ProcessedData, self.data_callback)

		self.alg_decision = AlgDecision()

		self.associated = [[0,1,2], [0,3,4], [1,3,5], [2,4,5]] # dij associated to each source


	def data_callback(self, data):

		## k-Nearest Neighbors

		### Tree prediction
		pred = np.zeros((4))
		for i in range(4):
			input_dij = np.array([data.dij[k] for k in self.associated[i]])
			input_dij = np.expand_dims(input_dij, axis=0)
			pred[i] = self.knn_model[i].predict(input_dij)

		selected = pred.astype(np.bool)

		### Publish topic with algorithm decision
		self.alg_decision.stamp = rospy.Time.now()
		self.alg_decision.selected = selected
		self.decision_publisher.publish(self.alg_decision)


	def main_loop(self):
		while not rospy.is_shutdown():
			rospy.spin()




if __name__ == '__main__':
	state_estimation_node = KNNDecisionNode()

	try:
		state_estimation_node.main_loop()
	except rospy.ROSInterruptException:
		pass
