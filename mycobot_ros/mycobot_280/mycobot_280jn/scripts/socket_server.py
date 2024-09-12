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

# Í£Ö¹±êÖ¾
stop_event = threading.Event()

def process_data(data):
    """ ´¦Àí×Ö·û´®Êý¾Ý²¢×ª»»Îª¸¡µãÊýÁÐ±í """
    try:
        float_data = [float(x) for x in data.strip('[]').split(',')]
        return float_data
    except ValueError as e:
        print(f"数据处理错误: {e}")
        return None

def get_transform(listener):
    """ »ñÈ¡ joint1 ºÍ target_frame µÄ±ä»» """
    try:
        # »ñÈ¡µ±Ç°Ê±¼ä
        now = rospy.Time.now()

        # ¼ì²éÊÇ·ñ´æÔÚ joint1 ºÍ target_frame
        if not listener.frameExists("joint1") or not listener.frameExists("target_frame"):
            print("joint1 或 target_frame 坐标系不存在")
            return None, None

        # ¼õÉÙµÈ´ý³¬Ê±Ê±¼ä
        listener.waitForTransform("joint1", "target_frame", now, rospy.Duration(0.1))
        # »ñÈ¡ joint1 ºÍ target_frame µÄ±ä»»
        (trans, rot) = listener.lookupTransform("joint1", "target_frame", now)
        return trans, rot
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print(f"获取变换失败: {e}")
        return None, None

def start_server(host, port, shared_data, data_lock):
    """ Æô¶¯ Socket ·þÎñÆ÷²¢½ÓÊÕÊý¾Ý """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"服务器启动，监听 {host}:{port}")
    
    listener = tf.TransformListener()

    while not stop_event.is_set():
        client_socket, client_address = server_socket.accept()
        print(f"客户端连接: {client_address}")

        try:
            while not stop_event.is_set():
                data = client_socket.recv(1024)
                if not data:
                    break
                message = data.decode('utf-8')
                print(f"接受到的数据: {message}")
                
                pro_data = process_data(message)
                if pro_data:
                    # ¼ì²éÊÇ·ñÊÕµ½ [1]
                    if pro_data == [1]:
                        # »ñÈ¡ joint1 ºÍ target_frame µÄ±ä»»
                        trans, rot = get_transform(listener)
                        if trans and rot:
                            # ½«±ä»»ÐÅÏ¢·¢ËÍ»Ø¿Í»§¶Ë
                            response_message = f"Translation: {trans}, Rotation: {rot}"
                            client_socket.sendall(response_message.encode('utf-8'))
                        else:
                            client_socket.sendall("获取变换失败".encode('utf-8'))
                    else:
                        # Õý³£¸üÐÂ×ø±êÊý¾Ý
                        with data_lock:
                            shared_data[:] = pro_data
                        print(f"更新后的坐标数据: {shared_data}")
                        client_socket.sendall(f"服务器收到: {message}".encode('utf-8'))

        except Exception as e:
            print(f"发生错误: {e}")

        finally:
            client_socket.close()
            print(f"客户端 {client_address} 已断开连接")

def publish_tf_broadcast(shared_data, data_lock):
    """ ·¢²¼ TF ¹ã²¥ """
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown() and not stop_event.is_set():
        with data_lock:
            current_data = list(shared_data)

        if len(current_data) == 6:
            # ÌáÈ¡Î»ÖÃºÍÐý×ª½Ç
            xyz = current_data[0:3]  # [x, y, z]
            rpy = current_data[3:6]  # [rx, ry, rz]

            # ½«Ðý×ª½Ç×ª»»ÎªËÄÔªÊý
            quaternion = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

            # ¹ã²¥×ø±ê±ä»»
            br.sendTransform(
                xyz, quaternion, rospy.Time.now(), "target_frame", "joint6_flange"
            )
            print(f"广播的坐标: xyz={xyz}, quaternion={quaternion}")

        rate.sleep()

def signal_handler(sig, frame):
    print('Received interrupt signal')
    stop_event.set()  # ÉèÖÃ±êÖ¾Í¨ÖªÏß³ÌÍË³ö
    sys.exit(0)  # ÍË³öÖ÷³ÌÐò

if __name__ == "__main__":

    # ÉèÖÃÖÐ¶ÏÐÅºÅ´¦ÀíÆ÷
    signal.signal(signal.SIGINT, signal_handler)
    
    ifname = "wlan0"
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    HOST = socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', bytes(ifname, encoding="utf8")))[20:24])
    PORT = 9000

    # Ê¹ÓÃ Manager ´´½¨¹²ÏíÁÐ±íºÍËø
    with Manager() as manager:
        shared_data = manager.list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        data_lock = manager.Lock()
        
        # ´´½¨ROS½Úµã
        rospy.init_node('target_tf_broadcaster', anonymous=True)
        
        # Æô¶¯socket·þÎñÆ÷Ïß³Ì
        server_thread = threading.Thread(target=start_server, args=(HOST, PORT, shared_data, data_lock))
        server_thread.start()

        # Æô¶¯·¢²¼TF¹ã²¥Êý¾ÝµÄÏß³Ì
        publish_data_thread = threading.Thread(target=publish_tf_broadcast, args=(shared_data, data_lock))
        publish_data_thread.start()

        try:
            rospy.spin()  # ±£³Ö ROS ½Úµã»îÔ¾
        except KeyboardInterrupt:
            print("Shutting down ROS node")
            rospy.signal_shutdown("KeyboardInterrupt")
