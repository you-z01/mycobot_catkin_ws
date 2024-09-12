#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def broadcast_static_transform():
    rospy.init_node('static_transform_broadcaster')

    # Create a StaticTransformBroadcaster object
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Define the static transform from the camera frame to the end-effector frame
    static_transform = geometry_msgs.msg.TransformStamped()
    
    # Frame IDs
    static_transform.header.frame_id = 'joint6_flange'  # Parent frame
    static_transform.child_frame_id = 'camera_link'  # Child frame
    
    # Time stamp
    static_transform.header.stamp = rospy.Time.now()

    # Set translation and rotation based on hand-eye calibration results
    # static_transform.transform.translation.x = -0.0291947
    # static_transform.transform.translation.y = -0.0481932
    # static_transform.transform.translation.z = 0.0133379

    # # Convert from rotation angles (rx, ry, rz) to quaternion
    # static_transform.transform.rotation.x = 0.016861922075053517
    # static_transform.transform.rotation.y = -0.0012176168900470685
    # static_transform.transform.rotation.z = 0.023756027719189287
    # static_transform.transform.rotation.w = 0.999574831685977
    static_transform.transform.translation.x = -0.0315452
    static_transform.transform.translation.y = -0.0547184
    static_transform.transform.translation.z = 0.00294731

    # x y z w
    static_transform.transform.rotation.x = 0.023915413473844602
    static_transform.transform.rotation.y = 0.019131787750649276
    static_transform.transform.rotation.z = 0.01410083587223854
    static_transform.transform.rotation.w = 0.9994314354289359



    # Broadcast the static transform
    static_broadcaster.sendTransform(static_transform)

    rospy.loginfo("Broadcasting static transform from 'end_link' to 'camera'")
    
    rospy.spin()

if __name__ == '__main__':
    broadcast_static_transform()


# pubish_aruco.py
# from geometry_msgs.msg import PoseStamped
# def pose_callback(msg):
#     # 解析接收到的Aruco标记位姿
#     pose = msg.pose
#     position = pose.position
#     orientation = pose.orientation
    
#     # 创建TF广播器
#     br = tf.TransformBroadcaster()
    
#     # 将Aruco标记的位姿从相机坐标系中广播
#     br.sendTransform(
#         (position.x, position.y, position.z),
#         (orientation.x, orientation.y, orientation.z, orientation.w),
#         rospy.Time.now(),
#         "aruco_marker",
#         "camera_link"  # 变换到相机坐标系
#     )

# def listener():
#     rospy.init_node('aruco_pose_listener', anonymous=True)
#     rospy.Subscriber('/aruco_single/pose', PoseStamped, pose_callback)
#     rospy.spin()

# if __name__ == '__main__':
#     listener()