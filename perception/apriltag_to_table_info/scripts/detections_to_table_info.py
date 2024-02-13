import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray

def process_info(msg):
	# tag id: [table_height, table_width, table_depth]
	id_to_info = {0:[.6, 3, .8], 1:[.6, 1, 1]}
	
	for detection in msg.detections:
		tag_id = detection.id[0]
		rospy.loginfo(f"apriltag id: {tag_id}")
		tag_info = id_to_info[tag_id]
		rospy.loginfo(f"height: {tag_info[0]}")
		rospy.loginfo(f"width: {tag_info[1]}")
		rospy.loginfo(f"depth: {tag_info[2]}")
		# to add info on height, width, depth
		
		tag_pose_relative_to_camera = detection.pose.pose.pose;
		# transform to PoseStamped
		tag_pose_relative_to_camera_stamped = geometry_msgs.msg.PoseStamped()
		tag_pose_relative_to_camera_stamped.header.stamp = rospy.Time.now()
		tag_pose_relative_to_camera_stamped.header.frame_id = "xtion_rgb_optical_frame"
		tag_pose_relative_to_camera_stamped.pose = tag_pose_relative_to_camera

		#rospy.loginfo(f"Pose relative to camera: {tag_pose_relative_to_camera}") # debugging
		
		try:
			transform = tf_buffer.lookup_transform("base_footprint", "xtion_rgb_optical_frame", rospy.Time(0), rospy.Duration(1.0))
			rospy.loginfo(f"transform: {transform}")
		except Exception as e:
			rospy.loginfo("failed to lookup transform")
			rospy.loginfo(e)

		tag_pose_relative_to_base_stamped = tf2_geometry_msgs.do_transform_pose(tag_pose_relative_to_camera_stamped, transform)
		tag_pose_relative_to_base = tag_pose_relative_to_base_stamped.pose

		rospy.loginfo(f"transformed pose: {tag_pose_relative_to_base}")
			

if __name__=='__main__':
	rospy.init_node("apriltag_detection_lister", anonymous=True)
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer)
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray,process_info)
	rospy.spin()

