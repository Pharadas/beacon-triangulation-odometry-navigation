#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import resource_retriever
import random

def talker():
    pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
    marker = Marker()

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        X = random.random() * 10 - 5
        Y = random.random() * 10 - 5
        Z = random.random() * 10 - 5
        
        
        
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/map"

        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 10
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # set marker orientation 
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0


        # set marker position
        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = Z

        point.point.x = X
        point.point.y = Y
        point.point.z = Z + 3

        marker.mesh_resource = "file:///home/berdehacks/lidar_ws/beacon-triangulation-odometry-navigation/src/ptcloudreader/beacon.stl"

        # Publish point and marker
        pub.publish(point)
        marker_pub.publish(marker)
        
        print(point)
        rate.sleep()

if __name__ == '__main__':
   try:
       talker()
   except rospy.ROSInterruptException:
       pass
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import random

def talker():
    pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "/map"
        point.point.x = random.random() * 10 - 5
        point.point.y = random.random() * 10 - 5
        point.point.z = random.random() * 10 - 5

        pub.publish(point)
        print(point)
        rate.sleep()

if __name__ == '__main__':
   try:
       talker()
   except rospy.ROSInterruptException:
       pass
