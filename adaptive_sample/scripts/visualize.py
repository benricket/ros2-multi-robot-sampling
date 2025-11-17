#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped

class Visualizer(Node):
    def __init__(self):
        super().__init__("matplotlib_visualizer")
        self.timer = self.create_timer(0.5,self.timer_callback)
        self.fig, self.ax = plt.subplots(1,1)
        plt.ion()
        
        # Initialize Subscribers
        self.field_sub = self.create_subscription(MarkerArray,"/model_vis",self.model_vis_callback,10)
        self.robot_pose_sub = self.create_subscription(PoseStamped,"/robot999/pos",self.robot_pose_callback,10)
        
        # fields to hold data
        self.model_data = np.zeros(shape=(100,100))
        self.robot_pose = [0.0,0.0]

    def model_vis_callback(self,msg: MarkerArray):
        res_x = 100
        res_y = 100
        data = np.zeros(shape=(res_x,res_y))
        for i,marker in enumerate(msg.markers):
            data[i % res_x,i // res_y] = marker.color.r
        self.model_data = data
    
    def robot_pose_callback(self,msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_pose = [x,y]
        
    def timer_callback(self):
        print(f"timer callback: pose is {self.robot_pose}")
        self.ax.imshow(self.model_data)
        self.ax.plot(self.robot_pose[0],self.robot_pose[1],'o',markersize=100)
        #self.ax.draw()
        plt.pause(0.001)

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(Visualizer())
    rclpy.shutdown()