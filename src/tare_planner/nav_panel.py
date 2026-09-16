#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tkinter as tk
from tkinter import messagebox
from geometry_msgs.msg import PointStamped
import threading

class WheeltecNavPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("Wheeltec Nav Console")
        self.root.geometry("350x250")
        self.root.attributes('-topmost', True)

        # ROS init
        rospy.init_node('wheeltec_nav_gui', anonymous=True)
        self.waypoint_pub = rospy.Publisher('/waypoint', PointStamped, queue_size=1)
        rospy.Subscriber('/clicked_point', PointStamped, self.rviz_click_callback)

        # UI variables
        self.target_x = tk.DoubleVar(value=0.0)
        self.target_y = tk.DoubleVar(value=0.0)

        # Threading buffers
        self.new_data_flag = False
        self.buffer_x = 0.0
        self.buffer_y = 0.0
        self.buffer_z = 0.0  # 新增：隐式保存真实的 Z 轴高度
        self.data_lock = threading.Lock()

        # Target Z for sending
        self.target_z = -0.4

        self.setup_ui()
        self.check_ros_and_update()

    def setup_ui(self):
        tk.Label(self.root, text="Target Coordinate Setting", font=("Arial", 14, "bold"), pady=10).pack()

        frame_x = tk.Frame(self.root)
        frame_x.pack(pady=5)
        tk.Label(frame_x, text="X (Forward) m: ", font=("Arial", 12)).pack(side=tk.LEFT)
        tk.Entry(frame_x, textvariable=self.target_x, font=("Arial", 12), width=10).pack(side=tk.LEFT)

        frame_y = tk.Frame(self.root)
        frame_y.pack(pady=5)
        tk.Label(frame_y, text="Y (Left) m: ", font=("Arial", 12)).pack(side=tk.LEFT)
        tk.Entry(frame_y, textvariable=self.target_y, font=("Arial", 12), width=10).pack(side=tk.LEFT)

        tk.Label(self.root, text="Tip: Use Publish Point in RViz to select target.", fg="gray").pack(pady=5)

        send_btn = tk.Button(self.root, text="Confirm and Launch", font=("Arial", 12, "bold"), bg="lightgreen", command=self.send_waypoint)
        send_btn.pack(pady=15, fill=tk.X, padx=50)

    def rviz_click_callback(self, msg):
        with self.data_lock:
            self.buffer_x = round(msg.point.x, 2)
            self.buffer_y = round(msg.point.y, 2)
            self.buffer_z = msg.point.z  # 获取真实的 Z 轴高度
            self.new_data_flag = True

    def check_ros_and_update(self):
        if rospy.is_shutdown():
            self.root.destroy()
            return

        with self.data_lock:
            if self.new_data_flag:
                self.target_x.set(self.buffer_x)
                self.target_y.set(self.buffer_y)
                self.target_z = self.buffer_z  # 更新待发送的 Z 轴
                self.new_data_flag = False

        self.root.after(100, self.check_ros_and_update)

    def send_waypoint(self):
        try:
            x_val = self.target_x.get()
            y_val = self.target_y.get()

            goal_msg = PointStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"
            goal_msg.point.x = x_val
            goal_msg.point.y = y_val
            # 强制设为 0.0。对于 CMU 这种基于地面的局部规划器，
            # 0.0 通常代表“当前运动平面”，这能保证算法 100% 接受该点。
            goal_msg.point.z = 0.0 

            self.waypoint_pub.publish(goal_msg)
            print("Waypoint sent: X={}, Y={}, Z=0.0".format(x_val, y_val))

        except tk.TclError:
            messagebox.showerror("Error", "Please enter valid numbers.")
if __name__ == '__main__':
    try:
        root = tk.Tk()
        app = WheeltecNavPanel(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
