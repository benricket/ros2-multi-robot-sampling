#!/usr/bin/env python3

import rclpy
import sys
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float64MultiArray

class Visualizer(Node):
    def __init__(self):
        super().__init__("matplotlib_visualizer")
        # fields to hold data
        self.mean_data = np.zeros(shape=(100,100))
        self.var_data = np.zeros(shape=(100,100))

        self.robot_pose = [0.0,0.0]
        self.robot_target = [0.0,0.0]

        self.timer = self.create_timer(0.01,self.timer_callback)
        self.fig, self.ax = plt.subplots(1,2)

        self.ax[0].set_title("Predicted Values")
        self.mean_drawn = self.ax[0].imshow(self.mean_data,extent=[0,10.0,0,10.0])
        self.robot_drawn, = self.ax[0].plot(self.robot_pose[0],self.robot_pose[1],'o',color='white',markersize=10)
        self.target_drawn, = self.ax[0].plot(self.robot_target[0],self.robot_target[1],'o',color='red',markersize=10)

        self.ax[1].set_title("Prediction Variance")
        self.var_drawn = self.ax[1].imshow(self.var_data,extent=[0,10.0,0,10.0])
        self.robot_drawn_2, = self.ax[1].plot(self.robot_pose[0],self.robot_pose[1],'o',color='white',markersize=10)
        self.target_drawn_2, = self.ax[1].plot(self.robot_target[0],self.robot_target[1],'o',color='red',markersize=10)

        plt.ion()
        plt.show()
        
        # Initialize Subscribers
        self.mean_sub = self.create_subscription(Float64MultiArray,"/model_mean",self.mean_callback,10)
        self.var_sub = self.create_subscription(Float64MultiArray,"/model_var",self.var_callback,10)
        self.robot_pose_sub = self.create_subscription(PoseStamped,"/robot999/pos",self.robot_pose_callback,10)
        self.robot_target_sub = self.create_subscription(PoseStamped,"/robot999/target",self.robot_target_callback,10)

    def old_model_vis_callback(self,msg: MarkerArray):
        res_x = 100
        res_y = 100
        data = np.zeros(shape=(res_x,res_y))
        for i,marker in enumerate(msg.markers):
            data[i % res_x,i // res_y] = marker.color.r
            #data[i % res_x, i // res_y] = i % res_x + i // res_y
        print(f"Got model data: {data}")
        print(f"Min data: {np.min(data)}")
        print(f"Max data: {np.max(data)}")
        self.mean_data = data
    
    def mean_callback(self,msg: Float64MultiArray):
        res_x = 100
        res_y = 100
        data = np.zeros(shape=(res_x,res_y))
        for i,val in enumerate(msg.data):
            x = i // res_x
            y = i % res_x
            data[res_y - 1 - y, x] = val
        print(f"Got model means: {data}")
        print(f"Min mean: {np.min(data)}")
        print(f"Max mean: {np.max(data)}")
        self.mean_data = data

    def var_callback(self,msg: Float64MultiArray):
        res_x = 100
        res_y = 100
        data = np.zeros(shape=(res_x,res_y))
        for i,val in enumerate(msg.data):
            x = i // res_x
            y = i % res_x
            data[res_y - 1 - y, x] = val
        print(f"Got model variances: {data}")
        print(f"Min var: {np.min(data)}")
        print(f"Max var: {np.max(data)}")
        self.var_data = data
    
    def robot_pose_callback(self,msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_pose = [x,y]

    def robot_target_callback(self,msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_target = [x,y]
        
    def timer_callback(self):
        print(f"timer callback: pose is {self.robot_pose}")

        vmin = np.min(self.mean_data)
        vmax = np.max(self.mean_data)
        if vmin == vmax: # We want to cover a range of color in case all our input is the same
            vmax = vmin + 1e-6
        self.mean_drawn.set_clim(vmin=vmin, vmax=vmax)
        self.mean_drawn.set_data(self.mean_data)

        vmin = np.min(self.var_data)
        vmax = np.max(self.var_data)
        if vmin == vmax: # We want to cover a range of color in case all our input is the same
            vmax = vmin + 1e-6
        self.var_drawn.set_clim(vmin=vmin, vmax=vmax)
        self.var_drawn.set_data(self.var_data)

        self.robot_drawn.set_data(self.robot_pose[0],self.robot_pose[1])
        self.target_drawn.set_data(self.robot_target[0],self.robot_target[1])
        self.robot_drawn_2.set_data(self.robot_pose[0],self.robot_pose[1])
        self.target_drawn_2.set_data(self.robot_target[0],self.robot_target[1])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(Visualizer())
    rclpy.shutdown()