#!/usr/bin/env python  
import numpy as np
import rospy
from std_msgs.msg import String, Int32
import random

from time import sleep
'''
1.Early response 
2.Delayed response 
3.Random movement
4.Abnormal suggestions
5.Moving too slow
6.Moving too fast
7.Inappropriate placement
8.Getting too close to human
9.Hesitation
10.Stutter motion
11.Freeze in motion
12.Non-optimal motion path
'''



class error_generator:
    def __init__(self) -> None:
        rospy.init_node("action_log")
        self.error_pub=rospy.Publisher("/error_log",String)
        self.error_display_pub=rospy.Publisher("/error_display",String)
        self.error_index_pub=rospy.Publisher('/error_index', Int32)
        self.error_dict={
                        0: "No Error",
                        1:"Delayed response", 
                        2:"Random movement",
                        3:"Abnormal suggestions",
                        4:"Moving too slow",
                        5:"Inappropriate placement",
                        6:"Not release",
                        7:"Hesitation",
                        8:"Stutter motion",
                        9:"Freeze in motion",
                        10:"Non-optimal motion path"}        
        self.error_sequence=[0, 0, 0, 0, 4, 0, 8, 10, 0, 6, 0, 0, 0, 0, 9, 2, 0, 3, 0, 0, 0, 1, 0, 0, 0, 7, 0, 0, 5, 0, 9, 6, 7, 0, 8, 0, 0, 0, 1, 0, 0, 0, 0, 0, 5, 0, 0, 0, 2, 0, 10, 0, 4, 0, 0, 0, 0, 0, 3, 0, 4, 0, 0, 3, 0, 0, 0, 0, 0, 0, 9, 0, 6, 0, 0, 0, 2, 8, 0, 7, 10, 0, 0, 5, 0, 0, 0, 1, 0, 0]
    
    
    def error_loop(self):
        print(len(self.error_sequence))
        cnt=0
        state=0
        '''
        while not rospy.is_shutdown():
            if state==0:
                self.error_index_pub.publish(cnt)
                error_index=self.error_sequence[cnt]
                error_type=self.error_dict[error_index]
                self.error_display_pub.publish("NEXT: "+error_type)
            else:
                self.error_display_pub.publish("START,"+error_type)



            if keyboard.is_pressed('a') and state==0:
                self.error_pub.publish("START,"+error_type)
                state=1
            
            if keyboard.is_pressed('a') and state==1:
                self.error_pub.publish("END, " +error_type)
                self.error_display_pub.publish("END,"+error_type)
                rospy.sleep(1)
                self.error_display_pub.publish("Complete")
                rospy.sleep(2)
                cnt+=1
                state=0
            if keyboard.is_pressed('q'):
                break
            rospy.sleep(0.2)

            

            x=input("Error type is "+error_type+" press when complete")
            if keyboard.is_pressed('q'):
                break




            rospy.spin()


        '''
        while(len(self.error_sequence)>0):
            self.error_index_pub.publish(cnt)
            error_index=self.error_sequence[cnt]
            error_type=self.error_dict[error_index]
            self.error_display_pub.publish("NEXT: "+error_type)
            x=input("Error type is "+error_type+" press when start")
            if x=="x":
                 break
            self.error_pub.publish("START,"+error_type)
            self.error_display_pub.publish("START,"+error_type)

            x=input("Error type is "+error_type+" press when complete")
            if x=="x":
                 break
            self.error_pub.publish("END, " +error_type)
            self.error_display_pub.publish("END,"+error_type)
            rospy.sleep(1)
            self.error_display_pub.publish("Complete")
            rospy.sleep(2)
            cnt+=1


        rospy.spin()
        


if __name__ == "__main__":
    error=error_generator()
    error.error_loop()
