#!/usr/bin python
import rospy
import tf2_ros
import geometry_msgs.msg

def track_aruco():
    rospy.init_node('aruco_tracker')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/mycobot/arm_cmd', geometry_msgs.msg.Transform, queue_size=10)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('joint6_flange', 'aruco_marker_frame', rospy.Time(0))

            # ½«transformÐÅÏ¢·¢²¼µ½»úÐµ±ÛµÄ¿ØÖÆ»°ÌâÉÏ
            pub.publish(trans.transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            rate.sleep()
            continue

        rate.sleep()

if __name__ == '__main__':
    track_aruco()
