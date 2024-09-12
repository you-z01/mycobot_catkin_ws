#!/usr/bin/env python3
# coding:utf-8

import rospy
import tf

def get_transform():
    rospy.init_node('get_transform_node', anonymous=True)
    listener = tf.TransformListener()

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        try:
            # 获取当前时间
            now = rospy.Time.now()
            # 等待变换
            listener.waitForTransform("joint1", "target_frame", now, rospy.Duration(1.0))
            # 获取 joint1 和 target_frame 的变换
            (trans, rot) = listener.lookupTransform("joint1", "target_frame", now)
            
            # 打印平移和旋转信息
            rospy.loginfo(f"Translation: {trans}")
            rospy.loginfo(f"Rotation (quaternion): {rot}")

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")

        rate.sleep()

if __name__ == "__main__":
    try:
        get_transform()
    except rospy.ROSInterruptException:
        pass
