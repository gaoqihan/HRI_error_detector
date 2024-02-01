#!/usr/bin/env python  
import numpy as np
import rospy
from std_msgs.msg import String
import random
class action_log_viewer:
    def __init__(self) -> None:
        rospy.init_node("action_log_viewer")

        error_sub=rospy.Subscriber("/error_behavior",String,callback=self.cb)

    def cb(self,msg:String):
        print(msg)

        return
if __name__ == "__main__":
    error=action_log_viewer()
    rate=rospy.Rate(20)

    while(1):
        rospy.sleep(rate)