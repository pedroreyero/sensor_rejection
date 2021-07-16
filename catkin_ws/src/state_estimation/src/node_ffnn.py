#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import os
import onnxruntime as rt
from state_estimation.msg import ProcessedData, AlgDecision


class FFNNDecisionNode:

	def __init__(self):

		rospy.init_node('node_ffnn', anonymous=True)

		r = rospkg.RosPack()
		path = os.path.join(r.get_path('state_estimation'), "models/ffnn_model.onnx")
		self.nn_model = rt.InferenceSession(path)

		self.decision_publisher = rospy.Publisher('state_estimation/ffnn_decision', AlgDecision, queue_size=1)

		self.data_sub = rospy.Subscriber('state_estimation/processed_data', ProcessedData, self.data_callback)

		self.alg_decision = AlgDecision()


	def data_callback(self, data):

		## Feed-forward FC NN

		### NN prediction
		nn_input = [data.cartographer_x, data.cartographer_y, data.cartographer_th, \
					data.gps_x, data.gps_y, data.gps_th, \
					data.dragonfly_x, data.dragonfly_y, data.dragonfly_th, \
					data.deadr_x, data.deadr_y, data.deadr_th]
		nn_input = np.array([nn_input]).astype(np.float32)
		nn_input = np.reshape(nn_input, (1, -1)) # from 1D to 2D array
		input_name = self.nn_model.get_inputs()[0].name
		label_name = self.nn_model.get_outputs()[0].name

		pred = self.nn_model.run([label_name], {input_name: nn_input})[0]
		pred = pred.squeeze() # from 2D to 1D array

		selected = (pred > 0.5)

		### Publish topic with algorithm decision
		self.alg_decision.stamp = rospy.Time.now()
		self.alg_decision.selected = selected
		self.alg_decision.trust = pred
		self.decision_publisher.publish(self.alg_decision)


	def main_loop(self):
		while not rospy.is_shutdown():
			rospy.spin()




if __name__ == '__main__':
	state_estimation_node = FFNNDecisionNode()

	try:
		state_estimation_node.main_loop()
	except rospy.ROSInterruptException:
		pass
