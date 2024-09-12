#!/usr/bin python
import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def broadcast_static_tf():
    rospy.init_node('tf_broadcaster')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transformStamped = geometry_msgs.msg.TransformStamped()

    transformStamped.header.stamp = rospy.Time.now()
    transformStamped.header.frame_id = "camera_color_frame"
    transformStamped.child_frame_id = "joint6_flange"

    # Ê¹ÓÃÊÖÑÛ±ê¶¨½á¹û
    transformStamped.transform.translation.x = -0.0291947
    transformStamped.transform.translation.y = -0.0481932
    transformStamped.transform.translation.z = 0.0133379

    q = tf_conversions.transformations.quaternion_from_euler(1.92847, -0.185372, 2.71977)
    transformStamped.transform.rotation.x = q[0]
    transformStamped.transform.rotation.y = q[1]
    transformStamped.transform.rotation.z = q[2]
    transformStamped.transform.rotation.w = q[3]

    broadcaster.sendTransform(transformStamped)

    rospy.spin()

if __name__ == '__main__':
    broadcast_static_tf()
