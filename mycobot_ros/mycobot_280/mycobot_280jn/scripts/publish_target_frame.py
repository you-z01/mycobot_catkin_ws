#!/usr/bin/env python3
# coding:utf-8

from pymycobot.mycobot import MyCobot
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord

import rospy
import tf
import threading
import socket
import fcntl
import struct
from multiprocessing import Manager
import signal
import sys
from geometry_msgs.msg import TransformStamped

mc = MyCobot("/dev/ttyTHS1", 1000000)
# 停止标志
stop_event = threading.Event()
latest_transform = None

def process_data(data):
    """ 处理字符串数据并转换为浮点数列表 """
    try:
        float_data = [float(x) for x in data.strip('[]').split(',')]
        return float_data
    except ValueError as e:
        print(f"数据处理错误: {e}")
        return None

def transform_callback(msg):
    """回调函数，接收并存储最新的变换消息"""
    global latest_transform
    latest_transform = msg

def start_server(host, port, shared_data, data_lock):
    
    """ 启动 Socket 服务器并接收数据 """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    # server_socket.settimeout(60)  # 设置超时时间为60秒
    # print(f"服务器启动，监听 {host}:{port}")
    rospy.logwarn(f"服务器启动，监听: {host}:{port}")

    while not stop_event.is_set():
  
        client_socket, client_address = server_socket.accept()
        rospy.logwarn(f"客户端连接: {client_address}")

        try:
            while not stop_event.is_set():
                data = client_socket.recv(1024)
                if not data:
                    break
                message = data.decode('utf-8')
                # print(f"接收到的数据: {message}")
                rospy.loginfo(f"接收到的数据: {message}")
                response_message = ""  # 确保变量初始化
                
                pro_data = process_data(message)
               
                if pro_data:
                    # 当接收到的数据为1时，查询消息返回信息
                    if len(pro_data) == 1 and pro_data == [1]:
                        #mc.send_angles([0, 0, 0, 0, 0, 0], 30)
                        global latest_transform
                        if latest_transform:
                            transform_data = [
                                latest_transform.transform.translation.x,
                                latest_transform.transform.translation.y,
                                latest_transform.transform.translation.z,
                                latest_transform.transform.rotation.x,
                                latest_transform.transform.rotation.y,
                                latest_transform.transform.rotation.z,
                                latest_transform.transform.rotation.w
                            ]
                            
                            response_message = str(transform_data)
                            client_socket.sendall(response_message.encode('utf-8'))
                            # print(f"获取到的joint1和target的变换数据：{response_message}")
                            rospy.logwarn(f"获取到的joint1和target的变换数据: {response_message}")

                        else:
                            print("没有收到最新的变换数据")
                            
                    # 说明是指令数据的控制数据（数据首尾校验）
                    elif pro_data[0] == 6 and pro_data[-1] == 9:
                        mess = cmd_control(len(pro_data), pro_data)
                        client_socket.sendall(str(mess).encode('utf-8'))

                    else:
                        with data_lock:
                            shared_data[:] = pro_data
                             #print(f"发送的变换数据：{response_message}")
                            rospy.logwarn(f"发送的变换数据：{response_message}")

                    # print(f"更新后的坐标数据: {shared_data}")
                    rospy.loginfo(f"当前的坐标数据: {shared_data}")

        except Exception as e:
            print(f"发生错误: {e}")

        finally:
            client_socket.close()
            # print(f"客户端 {client_address} 已断开连接")
            rospy.logwarn(f"客户端 {client_address} 已断开连接")

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


def cmd_control(length, list_data):
    #data = list_data[1:-1]  # 取出数据 [6,3,9] 则data为[3]
    if length == 3:
        # [6,2,9] 前后为校验位
        if list_data[1] == 1:
            s = "机械臂关电"
            mc.power_off()  # 机械臂关电
            return s
        elif list_data[1] == 2:
            s = "机械臂上电"
            mc.power_on()   # 机械臂上电
            return s
        elif list_data[1] == 3:
            s = "获取到机械臂位姿:"
            coor = mc.get_coords()
            s = s + str(coor)
            return s
        elif list_data[1] == 4:
            s = "获取到机械臂角度:"
            ang = mc.get_angles()
            s = s + str(ang)
            return s
        elif list_data[1] == 5:
            s = "关夹具:"
            mc.set_gripper_value(55, 20,1)
            return s
        elif list_data[1] == 6:
            s = "开夹具:"
            mc.set_gripper_value(100, 20,1)
            return s
        else:
            return "无效命令!"
    elif length == 4:
        # [6,55,15,9]
        s = "夹抓到达位置和速度:"
        mc.set_gripper_value(list_data[0], list_data[1], 1)
        s = s + str(list_data[0]) + "and" + str(list_data[1])
        return s  
    elif length == 9:
        # [6,30,0,0,0,0,0,0,9]  速度30,角度[0,0,0,0,0,0]
        speed = int(list_data[1])
        angle = list_data[2:8]
        rospy.loginfo(f"speed{speed}")
        rospy.loginfo(f"angle{angle}")
        mc.send_angles(angle, speed)
    else:
        s = "无效数据长度!"
        return s



       


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

        # 订阅/joint1_to_target_transform主题
        rospy.Subscriber('/joint1_to_target_transform', TransformStamped, transform_callback)
        
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

