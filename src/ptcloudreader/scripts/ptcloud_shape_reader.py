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
