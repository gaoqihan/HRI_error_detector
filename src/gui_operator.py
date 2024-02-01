#!/usr/bin/env python

import cv2
import rospy
import tkinter as tk
from tkinter import *
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
from PIL import Image as IMG
from PIL import ImageTk

class VideoDisplay:
    def __init__(self, root):
        self.cv_bridge = CvBridge()

        # Initialize ROS node
        rospy.init_node('gui_operator')

        # Define video subscribers
        self.sub_cam1 = rospy.Subscriber('/usb_cam1/image_raw', Image, self.callback_cam1)
        self.sub_cam2 = rospy.Subscriber('/camera/color/image_raw', Image, self.callback_cam2)

        # Define text message subscriber
        self.sub_message = rospy.Subscriber('/error_display', String, self.callback_message)
        self.sub_error_index = rospy.Subscriber('/error_index', Int32, self.callback_index)

        # Create tkinter window
        self.root = root
        self.root.title('Video Display')

        # Create labels for video and text display
        self.label_cam1 = tk.Label(root)
        self.label_cam2 = tk.Label(root)
        self.label_message = tk.Label(root, text="", font=("Helvetica", 60))
        self.label_message.grid(row=1,column=0,columnspan=20,sticky=(W,S,E,N),pady=2)

        # Initialize labels for future error display
        self.future_list_1 = tk.Label(root, text="",font=("Helvetica", 20))
        self.future_list_2 = tk.Label(root, text="",font=("Helvetica", 20))
        self.future_list_3 = tk.Label(root, text="",font=("Helvetica", 20))
        
        self.label_cam1.grid(row=2,column=0,rowspan=10,columnspan=10,sticky=W,padx=2,pady=2)

        self.label_cam2.grid(row=5,column=11,rowspan=10,columnspan=10,sticky=N,padx=2,pady=2)
        # Pack future error labels to the right of the right video window
        self.future_list_1.grid(row=2,column=11,columnspan=10,sticky=(N,W))
        self.future_list_2.grid(row=3,column=11,columnspan=10,sticky=(N,W))
        self.future_list_3.grid(row=4,column=11,columnspan=10,sticky=(N,W))

        # Pack labels to the window

        # Initialize bounding box color to default (no bounding box)
        self.bounding_box_color = None



        self.error_dict = {
            0: "No Error",
            1: "Delayed response",
            2: "Random movement",
            3: "Abnormal suggestions",
            4: "Moving too slow",
            5: "Inappropriate placement",
            6: "Not release",
            7: "Hesitation",
            8: "Stutter motion",
            9: "Freeze in motion",
            10: "Non-optimal motion path"
        }
        self.error_sequence=[0, 0, 0, 0, 4, 0, 8, 10, 0, 6, 0, 0, 0, 0, 9, 2, 0, 3, 0, 0, 0, 1, 0, 0, 0, 7, 0, 0, 5, 0, 9, 6, 7, 0, 8, 0, 0, 0, 1, 0, 0, 0, 0, 0, 5, 0, 0, 0, 2, 0, 10, 0, 4, 0, 0, 0, 0, 0, 3, 0, 4, 0, 0, 3, 0, 0, 0, 0, 0, 0, 9, 0, 6, 0, 0, 0, 2, 8, 0, 7, 10, 0, 0, 5, 0, 0, 0, 1, 0, 0]

    def callback_cam1(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        #img = cv2.resize(img, (400, 300), interpolation = cv2.INTER_NEAREST)
        img_tk = self.convert_to_tkinter_image(img)
        self.label_cam1.configure(image=img_tk)
        self.label_cam1.image = img_tk

    def callback_cam2(self, data):
        img = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        #img = cv2.resize(img, (1600, 1200), interpolation = cv2.INTER_NEAREST)
        img_tk = self.convert_to_tkinter_image(img)
        self.label_cam2.configure(image=img_tk)
        self.label_cam2.image = img_tk

    def callback_message(self, data):
        # Check if "START" is present in the received message
        if "START" in data.data:
            self.bounding_box_color = 'green'
        # Check if "END" is present in the received message
        elif "END" in data.data:
            self.bounding_box_color = 'red'
        else:
            self.bounding_box_color = None

        if "No Error" in data.data:
            self.bounding_box_color = None

        # Update the window with the new bounding box color
        self.update_bounding_box()

        # Update label with text message
        self.label_message.configure(text="{}".format(data.data))

    def callback_index(self, data):
        self.future_list_1.configure(text="1: {}".format(self.error_dict[self.error_sequence[data.data + 1]]))
        self.future_list_2.configure(text="2: {}".format(self.error_dict[self.error_sequence[data.data + 2]]))
        self.future_list_3.configure(text="3: {}".format(self.error_dict[self.error_sequence[data.data + 3]]))

    def convert_to_tkinter_image(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = IMG.fromarray(img)
        img_tk = ImageTk.PhotoImage(img)
        return img_tk

    def update_bounding_box(self):
        # Update the window with the new bounding box color
        if self.bounding_box_color:
            self.root.configure(bg=self.bounding_box_color)
        else:
            # If no specific color, set to default background color
            self.root.configure(bg='white')

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    root = tk.Tk()
    video_display = VideoDisplay(root)
    video_display.run()
