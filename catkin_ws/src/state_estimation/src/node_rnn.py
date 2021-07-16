#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import os
import onnxruntime as rt
from state_estimation.msg import ProcessedData, AlgDecision


class RNNDecisionNode:

	def __init__(self):

		rospy.init_node('node_rnn', anonymous=True)

		r = rospkg.RosPack()
		path = os.path.join(r.get_path('state_estimation'), "models/rnn_model.onnx")
		self.nn_model = rt.InferenceSession(path)

		self.decision_publisher = rospy.Publisher('state_estimation/rnn_decision', AlgDecision, queue_size=1)

		self.data_sub = rospy.Subscriber('state_estimation/processed_data', ProcessedData, self.data_callback)

		self.alg_decision = AlgDecision()

		Tx = 300
		input_size = self.nn_model.get_inputs()[0].shape[2]
		self.last_inputs = np.zeros((1,Tx,input_size))


	def data_callback(self, data):

		## Recurrent NN

		### NN prediction
		nn_input = [data.cartographer_x, data.cartographer_y, data.cartographer_th, \
					data.gps_x, data.gps_y, data.gps_th, \
					data.dragonfly_x, data.dragonfly_y, data.dragonfly_th, \
					data.deadr_x, data.deadr_y, data.deadr_th]
		self.last_inputs = np.roll(self.last_inputs, -1, axis=1)
		self.last_inputs[:,-1,:] = nn_input
		input_name = self.nn_model.get_inputs()[0].name
		label_name = self.nn_model.get_outputs()[0].name

		pred = self.nn_model.run([label_name], {input_name: self.last_inputs.astype(np.float32)})[0]
		pred = pred[0,-1,:]

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
	state_estimation_node = RNNDecisionNode()

	try:
		state_estimation_node.main_loop()
	except rospy.ROSInterruptException:
		pass
