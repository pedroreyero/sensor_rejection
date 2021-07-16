#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from state_estimation.msg import ProcessedData
import numpy as np
from lltoutm import LLtoUTM
from unwrap import unwrap
from proparker_msgs.msg import TruckState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32


class DataProcessingNode:

	def __init__(self):

		rospy.init_node('node_data', anonymous=True)

		self.data_publisher = rospy.Publisher('state_estimation/processed_data', ProcessedData, queue_size=1)

		self.ins_sub = rospy.Subscriber('ins_raw/state', TruckState, self.ins_callback)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
		self.cartographer_sub = rospy.Subscriber('cartographer/tracked_pose', PoseWithCovarianceStamped, self.cartographer_callback)
		self.dragonfly_sub = rospy.Subscriber('dragonfly_manager/tracked_pose', PoseWithCovarianceStamped, self.dragonfly_callback)
		self.gps_sub = rospy.Subscriber('gps', NavSatFix, self.gps_callback)

		self.reset_origins_srv = rospy.Service('state_estimation/reset_origins', Empty, self.handle_reset_origins)

		self.processed_data = ProcessedData()

		self.debug_pub = rospy.Publisher('state_estimation/debug', Float32, queue_size=1)

		self.ins_available = False
		self.ins_data = None
		self.odom_available = False
		self.odom_data = None
		self.last_deadr_d = 0
		self.deadr_x = 0
		self.deadr_y = 0
		self.deadr_refth = None
		self.deadr_th = None # prev th for unwrappping
		self.cartographer_available = False
		self.cartographer_data = None
		self.cartographer_turn = None
		self.cartographer_refx = None
		self.cartographer_refy = None
		self.cartographer_refth = None
		self.cartographer_th = None # prev th for unwrappping
		self.dragonfly_available = False
		self.dragonfly_data = None
		self.dragonfly_turn = None
		self.dragonfly_refx = None
		self.dragonfly_refy = None
		self.dragonfly_refth = None
		self.dragonfly_th = None  # prev th for unwrappping
		self.gps_available = False
		self.gps_data = None
		self.gps_turn = None
		self.gps_refx = None
		self.gps_refy = None
		self.gps_refth = None
		self.gps_th = None # prev th for unwrappping


	def ins_callback(self, data):

		self.ins_data = data
		self.ins_available = True

		self.check_and_fuse()


	def odom_callback(self, data):

		self.odom_data = data
		self.odom_available = True

		self.check_and_fuse()


	def cartographer_callback(self, data):

		self.cartographer_data = data
		self.cartographer_available = True

		self.check_and_fuse()


	def dragonfly_callback(self, data):

		self.dragonfly_data = data
		self.dragonfly_available = True

		self.check_and_fuse()


	def gps_callback(self, data):

		self.gps_data = data
		self.gps_available = True

		self.check_and_fuse()


	def handle_reset_origins(self, req):

		#### Make sure data is available
		if (self.ins_data is None) or (self.gps_data is None) or (self.odom_data is None) or \
				(self.cartographer_data is None) or (self.dragonfly_data is None):
				rospy.logerr("Reset origins was called when no data is available from the required topics!!")
				return EmptyResponse()


		#### If data is indeed available (expected behaviour):

		heading = self.ins_data.heading

		#### FIXED TRANSFORMATION
		## GPS origin
		UTMNorthing, UTMEasting, UTMZone = LLtoUTM(self.gps_data.latitude, self.gps_data.longitude)
		self.gps_turn = heading - np.pi/2
		self.gps_refx = UTMEasting
		self.gps_refy = UTMNorthing
		self.gps_refth = -heading # there is no way to unwrap upon initialization

		## Dead reckoning "origin"
		self.last_deadr_d = self.odom_data.pose.pose.position.x
		self.deadr_x = 0
		self.deadr_y = 0
		q = self.odom_data.pose.pose.orientation
		eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.deadr_refth = -eul[2] # there is no way to unwrap upon initialization
		self.deadr_th = self.deadr_refth # prev th for unwrappping

		## Cartographer origin
		q = self.cartographer_data.pose.pose.orientation
		eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.cartographer_turn = - eul[2]
		self.cartographer_refx = self.cartographer_data.pose.pose.position.x
		self.cartographer_refy = self.cartographer_data.pose.pose.position.y
		q = self.cartographer_data.pose.pose.orientation
		eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.cartographer_refth = eul[2] # there is no way to unwrap upon initialization
		self.cartographer_th = self.cartographer_refth # prev th for unwrappping

		## Dragonfly origin
		q = self.dragonfly_data.pose.pose.orientation
		eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
		self.dragonfly_turn = - eul[2]
		self.dragonfly_refx = self.dragonfly_data.pose.pose.position.x
		self.dragonfly_refy = self.dragonfly_data.pose.pose.position.y
		self.dragonfly_refth = eul[2] # there is no way to unwrap upon initialization
		self.dragonfly_th = self.dragonfly_refth # prev th for unwrappping


		return EmptyResponse()


	def check_and_fuse(self):

		if (self.ins_available and self.odom_available and self.cartographer_available and self.dragonfly_available and self.gps_available):
			# Everything available :)

			#### FIXED FRAME TRANSFORMATIONS
			## Process INS
			heading = self.ins_data.heading

			## Process GPS
			UTMNorthing, UTMEasting, UTMZone = LLtoUTM(self.gps_data.latitude, self.gps_data.longitude)
			if self.gps_turn is None:
				self.gps_turn = heading - np.pi/2
				self.gps_refx = UTMEasting
				self.gps_refy = UTMNorthing
				self.gps_refth = -heading # there is no way to unwrap upon initialization
				self.gps_th = self.gps_refth # prev th for unwrappping
			mat_turn = np.array([[np.cos(self.gps_turn), -np.sin(self.gps_turn)],[np.sin(self.gps_turn), np.cos(self.gps_turn)]])
			gps_x, gps_y = (np.matmul(mat_turn,np.array([[UTMEasting - self.gps_refx],[UTMNorthing-self.gps_refy]]))).ravel()
			gps_th = -heading
			gps_th = unwrap(gps_th, self.gps_th) - self.gps_refth
			self.gps_th = gps_th # prev th for wrapping
			CovGPS = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.001]])

			## Process Dead reckoning
			if self.deadr_refth is None:
				self.last_deadr_d = self.odom_data.pose.pose.position.x
				q = self.odom_data.pose.pose.orientation
				eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
				self.deadr_refth = -eul[2] # there is no way to unwrap upon initialization
				self.deadr_th = self.deadr_refth # prev th for wrapping
			deadr_d = self.odom_data.pose.pose.position.x
			delta_d = deadr_d - self.last_deadr_d
			self.last_deadr_d = deadr_d
			CovDR = np.array([[0.2,0,0],[0,0.2,0],[0,0,0.001]])

			deadr_turn = -(heading - np.pi/2) + self.gps_turn
			mat_turn = np.array([[np.cos(deadr_turn), -np.sin(deadr_turn)],[np.sin(deadr_turn), np.cos(deadr_turn)]])
			corrected = np.matmul(mat_turn, np.array([[delta_d],[0]]))
			self.deadr_x += corrected[0,0]
			self.deadr_y += corrected[1,0]
			deadr_x, deadr_y = self.deadr_x, self.deadr_y
			q = self.odom_data.pose.pose.orientation
			eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
			deadr_th = -eul[2]
			deadr_th = unwrap(deadr_th, self.deadr_th) - self.deadr_refth
			self.deadr_th = deadr_th # prev th for wrapping

			## Process Cartographer
			if self.cartographer_turn is None:
				q = self.cartographer_data.pose.pose.orientation
				eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
				self.cartographer_turn = - eul[2]
				self.cartographer_refx = self.cartographer_data.pose.pose.position.x
				self.cartographer_refy = self.cartographer_data.pose.pose.position.y
				self.cartographer_refth = eul[2] # there is no way to unwrap upon initialization
				self.cartographer_th = self.cartographer_refth # prev th for wrapping
			mat_turn = np.array([[np.cos(self.cartographer_turn), -np.sin(self.cartographer_turn)], \
									[np.sin(self.cartographer_turn), np.cos(self.cartographer_turn)]])
			cartographer_x, cartographer_y = (np.matmul(mat_turn,np.array( \
						[[self.cartographer_data.pose.pose.position.x - self.cartographer_refx], \
						[self.cartographer_data.pose.pose.position.y - self.cartographer_refy]]))).ravel()
			q = self.cartographer_data.pose.pose.orientation
			eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
			cartographer_th = eul[2]
			cartographer_th = unwrap(cartographer_th, self.cartographer_th) - self.cartographer_refth
			self.cartographer_th = cartographer_th # prev th for wrapping
			CovCG = np.array([[0.01,0,0],[0,0.01,0],[0,0,0.001]])

			## Process Dragonfly
			if self.dragonfly_turn is None:
				q = self.dragonfly_data.pose.pose.orientation
				eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
				self.dragonfly_turn = - eul[2]
				self.dragonfly_refx = self.dragonfly_data.pose.pose.position.x
				self.dragonfly_refy = self.dragonfly_data.pose.pose.position.y
				self.dragonfly_refth = eul[2] # there is no way to unwrap upon initialization
				self.dragonfly_th = self.dragonfly_refth # prev th for wrapping
			mat_turn = np.array([[np.cos(self.dragonfly_turn), -np.sin(self.dragonfly_turn)], \
									[np.sin(self.dragonfly_turn), np.cos(self.dragonfly_turn)]])
			dragonfly_x, dragonfly_y = (np.matmul(mat_turn,np.array( \
						[[self.dragonfly_data.pose.pose.position.x - self.dragonfly_refx], \
						[self.dragonfly_data.pose.pose.position.y - self.dragonfly_refy]]))).ravel()
			q = self.dragonfly_data.pose.pose.orientation
			eul = euler_from_quaternion([q.x, q.y, q.z, q.w])
			dragonfly_th = eul[2]
			dragonfly_th = unwrap(dragonfly_th, self.dragonfly_th) - self.dragonfly_refth
			self.dragonfly_th = dragonfly_th # prev th for wrapping
			CovDF = np.array([[0.02,0,0],[0,0.02,0],[0,0,0.001]])

			## ExNIS parity relations computation
			dX = np.array([cartographer_x, gps_x, dragonfly_x, deadr_x])
			dY = np.array([cartographer_y, gps_y, dragonfly_y, deadr_y])
			dTh = np.array([cartographer_th, gps_th, dragonfly_th, deadr_th])

			dXYTh = np.stack((dX,dY,dTh))
			I12 = CovCG + CovGPS;
			aux = (dXYTh[:,0] - dXYTh[:,1])[np.newaxis]
			d12 = np.matmul(np.matmul(aux,np.linalg.inv(I12)),aux.T).squeeze()
			I13 = CovCG + CovDF;
			aux = (dXYTh[:,0] - dXYTh[:,2])[np.newaxis]
			d13 = np.matmul(np.matmul(aux,np.linalg.inv(I13)),aux.T).squeeze()
			I14 = CovCG + CovDR;
			aux = (dXYTh[:,0] - dXYTh[:,3])[np.newaxis]
			d14 = np.matmul(np.matmul(aux,np.linalg.inv(I14)),aux.T).squeeze()
			I23 = CovGPS + CovDF;
			aux = (dXYTh[:,1] - dXYTh[:,2])[np.newaxis]
			d23 = np.matmul(np.matmul(aux,np.linalg.inv(I23)),aux.T).squeeze()
			I24 = CovGPS + CovDR;
			aux = (dXYTh[:,1] - dXYTh[:,3])[np.newaxis]
			d24 = np.matmul(np.matmul(aux,np.linalg.inv(I24)),aux.T).squeeze()
			I34 = CovDF + CovDR;
			aux = (dXYTh[:,2] - dXYTh[:,3])[np.newaxis]
			d34 = np.matmul(np.matmul(aux,np.linalg.inv(I34)),aux.T).squeeze()


			## Publish processed data
			self.processed_data.stamp = rospy.Time.now()

			self.processed_data.cartographer_x = cartographer_x
			self.processed_data.cartographer_y = cartographer_y
			self.processed_data.cartographer_th = cartographer_th
			self.processed_data.cartographer_cov = CovCG.reshape(-1,).tolist()

			self.processed_data.gps_x = gps_x
			self.processed_data.gps_y = gps_y
			self.processed_data.gps_th = gps_th
			self.processed_data.gps_cov = CovGPS.reshape(-1,).tolist()

			self.processed_data.dragonfly_x = dragonfly_x
			self.processed_data.dragonfly_y = dragonfly_y
			self.processed_data.dragonfly_th = dragonfly_th
			self.processed_data.dragonfly_cov = CovDF.reshape(-1,).tolist()

			self.processed_data.deadr_x = deadr_x
			self.processed_data.deadr_y = deadr_y
			self.processed_data.deadr_th = deadr_th
			self.processed_data.deadr_cov = CovDR.reshape(-1,).tolist()

			self.processed_data.dij = [d12, d13, d14, d23, d24, d34]

			self.data_publisher.publish(self.processed_data)


			#### RESET FLAGS
			self.ins_available = False
			self.odom_available = False
			self.cartographer_available = False
			self.dragonfly_available = False
			self.gps_available = False

			#### DEBUGGING PUBLISHER :)
			self.debug_pub.publish(Float32(gps_th))

		else:
			return


	def main_loop(self):
		while not rospy.is_shutdown():
			rospy.spin()




if __name__ == '__main__':
	state_estimation_node = DataProcessingNode()

	try:
		state_estimation_node.main_loop()
	except rospy.ROSInterruptException:
		pass
