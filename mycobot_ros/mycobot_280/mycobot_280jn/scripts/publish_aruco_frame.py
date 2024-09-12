#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
import tf

def talker():
    rospy.init_node("following_marker", anonymous=True)

    pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rate = rospy.Rate(20)
    listener = tf.TransformListener()

    marker_ = Marker()
    marker_.header.frame_id = "joint1"
    marker_.ns = "aruco_marker_frame"
    marker_.type = marker_.CUBE
    marker_.action = marker_.ADD
    marker_.scale.x = 0.04
    marker_.scale.y = 0.04
    marker_.scale.z = 0.04
    marker_.color.a = 1.0
    marker_.color.g = 1.0

    

    print("Publishing ...")
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        try:
            listener.waitForTransform("joint1", "aruco_marker_frame", now, rospy.Duration(1.0))
            trans, rot = listener.lookupTransform("joint1", "aruco_marker_frame", now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Exception: %s", str(e))
            continue

        marker_.header.stamp = now
        marker_.pose.position.x = trans[0]
        marker_.pose.position.y = trans[1]
        marker_.pose.position.z = trans[2]
        marker_.pose.orientation.x = rot[0]
        marker_.pose.orientation.y = rot[1]
        marker_.pose.orientation.z = rot[2]
        marker_.pose.orientation.w = rot[3]

        pub_marker.publish(marker_)
        #rospy.logwarn("marker_: %s", str(marker_))
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
