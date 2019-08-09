
import rospy, hebi_control, csv, tf

import laser_geometry as lg

from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header



def callback(msg):
	global hebiForLidar, br, lp, pubVal

	
	if rospy.is_shutdown():
		hebiForLidar.killswitch = True
	else:
		time = rospy.get_time()
		angle = eval(str(hebiForLidar.get_curr_angle()))[0] - hebiForLidar.angleBias
		scan = msg.ranges

		# pubVal.header = Header()
		# pubVal.header.stamp = rospy.Time.now()
		# pubVal.header.frame_id = "/laser_frame"
		# quat = tf.transformations.quaternion_from_euler(0, angle, 0)
		# pubVal.orientation = Quaternion()
		# pubVal.orientation.x = quat[0]
		# pubVal.orientation.y = quat[1]
		# pubVal.orientation.z = quat[2]
		# pubVal.orientation.w = quat[3]


		# publish_imu(pubVal)


		br.sendTransform((0, 0, 0), 
			tf.transformations.quaternion_from_euler(angle, 0, 0),
			rospy.Time.now(),
			"/laser_frame",
			"/camera_init"
			)



		# br.sendTransform((0, 0, 0), 
		# 	tf.transformations.quaternion_from_euler(angle, 0, 0),
		# 	rospy.Time.now(),
		# 	"/laser_frame",
		# 	"/base_footprint"
		# 	)
		# br.sendTransform((0, 0, 0), 
		# 	tf.transformations.quaternion_from_euler(angle, 0, 0),
		# 	rospy.Time.now(),
		# 	"/camera",
		# 	"/base_footprint"
		# 	)

	CSV_MODE = False

	if not CSV_MODE:
		pc2_msg = lp.projectLaser(msg)

		publish_pc2(pc2_msg)

	else:

		lines.append((str(time), str(angle)) + scan)

		if len(lines) >= 5:
			write_to_csv()


def publish_imu(pubVal):
	global pub2

	if not rospy.is_shutdown():

		pub2.publish(pubVal)


def publish_pc2(msg):
	global pub

	if not rospy.is_shutdown():
		pub.publish(msg)


def write_to_csv():
	global lines
	# Write to csv1
	with open("log.csv", 'a') as csvfile:
		csv_write = csv.writer(csvfile, delimiter=',')
		for line in lines:
			csv_write.writerow(line)
		csvfile.close()

	lines = []

	print("Wrote to csv 1")


if __name__ == '__main__':
	global hebiForLidar, lines, br, lp, pub, pubVal, pub2

	pubVal = Imu()

	emptyVelocity = Vector3()
	emptyVelocity.x = 0
	emptyVelocity.y = 0
	emptyVelocity.z = 0


	arr = [0 for _ in range(9)]

	pubVal.orientation_covariance = arr
	pubVal.angular_velocity_covariance = arr
	pubVal.linear_acceleration_covariance = arr

	pubVal.angular_velocity = emptyVelocity
	pubVal.linear_acceleration = emptyVelocity


	br = tf.TransformBroadcaster()

	lp = lg.LaserProjection()

	lines = []

	rospy.init_node("yahyeet", anonymous=True)
	rate = rospy.Rate(100)

	pub = rospy.Publisher('/sync_scan_cloud_filtered', PointCloud2, queue_size=10)
	pub2 = rospy.Publisher('/imu/data', Imu, queue_size=1)

	print("here1")
	hebiForLidar = hebi_control.HebiForLidar(2.5, 3.14159265 / 2, -3.14159265 / 2, -1.5, 'D8:80:39:EE:C1:CD')
	print("here2")
	rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)

	rospy.spin()