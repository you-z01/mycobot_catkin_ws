#!/usr/bin/env python
import rospy
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import tf
from tf.broadcaster import TransformBroadcaster
import tf_conversions
from mycobot_communication.srv import (
    GetCoords,
    SetCoords,
    GetAngles,
    SetAngles,
    GripperStatus,
)


class ImageConverter:
 
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        # Initialize ArUco parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters()

    def camera_info_callback(self, msg):
        # 从camera_info消息中提取相机矩阵和畸变系
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)
        rospy.loginfo("Camera parameters loaded.")

    


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        size = cv_image.shape
        focal_length = size[1]
        center = [size[1] / 2, size[0] / 2]
        if self.camera_matrix is None:
            self.camera_matrix = np.array(
                [
                    [focal_length, 0, center[0]],
                    [0, focal_length, center[1]],
                    [0, 0, 1],
                ],
                dtype=np.float32,
            )

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        ret = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        corners, ids = ret[0], ret[1]

        if len(corners) > 0:
            if ids is not None:
                ret = cv.aruco.estimatePoseSingleMarkers(
                    corners, 0.05, self.camera_matrix, self.dist_coeffs
                )
                (rvec, tvec) = (ret[0], ret[1])
                print("rvec:", rvec, "tvec:", tvec)
                for i in range(rvec.shape[0]):
                    aruco.drawDetectedMarkers(cv_image, corners)

                    cv.drawFrameAxes(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec[i, :, :],
                        tvec[i, :, :],
                        0.03,
                    )
                xyz = tvec[0, 0, :]
                xyz = [xyz[0] - 0.045, xyz[1], xyz[2] - 0.03]
                euler = rvec[0, 0, :]
                tf_change = tf.transformations.quaternion_from_euler(
                    euler[0], euler[1], euler[2]
                )
                print("tf_change:", tf_change)
                self.br.sendTransform(
                    xyz, tf_change, rospy.Time.now(), "aruco_marker_frame", "joint6_flange"
                )

        cv.imshow("Image", cv_image)
        cv.waitKey(3)


if __name__ == "__main__":
    try:
        rospy.init_node("realsense_detect_marker")
        rospy.loginfo("Starting realsense_detect_marker node")
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down realsense_detect_marker node.")
        cv.destroyAllWindows()

