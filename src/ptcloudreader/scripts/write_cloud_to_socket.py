#! /usr/bin/env python

import socket
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 1733))
server_socket.listen(1)
(client_socket, server_socket) = server_socket.accept()

def callback(msg):
    global server_socket
    global client_socket

    try:
        client_socket.send(msg.data)
    except:
        print("waiting for connection")
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 1733))
        server_socket.listen(1)
        (client_socket, server_socket) = server_socket.accept()
    print("sending data")

def listener():
    topic = rospy.get_param('~topic', '/yrl_pub/yrl_cloud')
    rospy.Subscriber(topic, PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pylistener', anonymous = True)
    listener()
