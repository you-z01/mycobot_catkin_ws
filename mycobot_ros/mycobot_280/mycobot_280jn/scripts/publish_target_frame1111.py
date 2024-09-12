#!/usr/bin/env python3
# coding:utf-8

import rospy
import tf
import threading
import socket
import fcntl
import struct
from multiprocessing import Manager
import signal
import sys

# 停止标志
stop_event = threading.Event()

def process_data(data):
    """ 处理字符串数据并转换为浮点数列表 """
    try:
        float_data = [float(x) for x in data.strip('[]').split(',')]
        return float_data
    except ValueError as e:
        print(f"数据处理错误: {e}")
        return None

def start_server(host, port, shared_data, data_lock):
    
    """ 启动 Socket 服务器并接收数据 """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"服务器启动，监听 {host}:{port}")

    while not stop_event.is_set():
        client_socket, client_address = server_socket.accept()
        print(f"客户端连接: {client_address}")

        try:
            while not stop_event.is_set():
                data = client_socket.recv(1024)
                if not data:
                    break
                message = data.decode('utf-8')
                print(f"接收到的数据: {message}")
                
                pro_data = process_data(message)
                if pro_data:
                    with data_lock:
                        shared_data[:] = pro_data
                    print(f"更新后的坐标数据: {shared_data}")

        except Exception as e:
            print(f"发生错误: {e}")

        finally:
            client_socket.close()
            print(f"客户端 {client_address} 已断开连接")

def publish_tf_broadcast(shared_data, data_lock):
    """ 发布 TF 广播 """
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown() and not stop_event.is_set():
        with data_lock:
            current_data = list(shared_data)

        if len(current_data) == 6:
            # 提取位置和欧拉角
            xyz = current_data[0:3]  # [x, y, z]
            rpy = current_data[3:6]  # [rx, ry, rz]

            # 将欧拉角转换为四元数
            quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

            # 广播位置变换
            br.sendTransform(
                xyz, quaternion, rospy.Time.now(), "target_frame", "joint6_flange"
            )
            # print(f"广播的TF坐标: xyz={xyz}, quaternion={quaternion}")

        rate.sleep()

def signal_handler(sig, frame):
    print('Received interrupt signal')
    stop_event.set()  # 设置标志通知线程退出
    sys.exit(0)  # 退出主程序


if __name__ == "__main__":

    # 设置中断信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    ifname = "wlan0"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    HOST = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', bytes(ifname, encoding="utf8")))[20:24])
    PORT = 9000

    # 使用 Manager 创建共享列表和锁
    with Manager() as manager:
        shared_data = manager.list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        data_lock = manager.Lock()
        
        # 创建ROS节点
        rospy.init_node('target_tf_broadcaster', anonymous=True)
        
        # 启动socket服务器线程
        server_thread = threading.Thread(target=start_server, args=(HOST, PORT, shared_data, data_lock))
        server_thread.start()

        # 启动发布TF广播数据的线程
        publish_data_thread = threading.Thread(target=publish_tf_broadcast, args=(shared_data, data_lock))
        publish_data_thread.start()

        try:
            rospy.spin()  # 保持 ROS 节点活动
        except KeyboardInterrupt:
            print("Shutting down ROS node")
            rospy.signal_shutdown("KeyboardInterrupt")

