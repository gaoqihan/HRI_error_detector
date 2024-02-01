import numpy as np
import rospy
from std_msgs.msg import String
rospy.init_node("user_rating")
error_pub=rospy.Publisher("/user_rating",String)

while(1):
    x=input("ratings?")
    if x=='x':
        break
    error_pub.publish(x)
