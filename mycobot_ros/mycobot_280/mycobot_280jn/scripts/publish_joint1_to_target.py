#!/usr/bin/env python3
# coding:utf-8

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def publish_transform():
    rospy.init_node('publish_transform_node', anonymous=True)
    listener = tf.TransformListener()
    transform_pub = rospy.Publisher('/joint1_to_target_transform', TransformStamped, queue_size=10)

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
            # rospy.loginfo(f"Translation: {trans}")
            # rospy.loginfo(f"Rotation (quaternion): {rot}")

            # 创建 TransformStamped 消息
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = "joint1"
            transform_msg.child_frame_id = "target_frame"
            transform_msg.transform.translation.x = trans[0]
            transform_msg.transform.translation.y = trans[1]
            transform_msg.transform.translation.z = trans[2]
            transform_msg.transform.rotation.x = rot[0]
            transform_msg.transform.rotation.y = rot[1]
            transform_msg.transform.rotation.z = rot[2]
            transform_msg.transform.rotation.w = rot[3]

            # 发布变换消息
            transform_pub.publish(transform_msg)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {e}")

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass
