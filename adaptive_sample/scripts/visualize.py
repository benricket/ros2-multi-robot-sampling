#!/usr/bin/env python3

import rclpy
import sys
import math
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
        self.ground_truth = np.zeros(shape=(100,100))
        self.cost_unweighted_data = np.zeros(shape=(100,100))
        self.cost_data = np.zeros(shape=(100,100))
        
        # For now, we're still using the exponential at 5,5 as our ground truth
        # Later we'll add a separate class or separate functions to hold our gnd truth
        
        # would be faster with vectorized np functions but it's a one time cost regardless
        for row in range(100):
            for col in range(100):
                self.ground_truth[row,col] = math.exp(-0.25*(pow(col/10 - 5.0,2)+3.0*pow(row/10 - 5.0,2)))

        self.robot_pose = [0.0,0.0]
        self.robot_target = [0.0,0.0]

        self.timer = self.create_timer(0.01,self.timer_callback)
        self.fig, self.ax = plt.subplots(1,5)

        self.ax[0].set_title("Predicted Values")
        self.mean_drawn = self.ax[0].imshow(self.mean_data,extent=[0,10.0,0,10.0])
        self.robot_drawn, = self.ax[0].plot(self.robot_pose[0],self.robot_pose[1],'o',color='white',markersize=10)
        self.target_drawn, = self.ax[0].plot(self.robot_target[0],self.robot_target[1],'o',color='red',markersize=10)

        self.ax[1].set_title("Prediction Variance")
        self.var_drawn = self.ax[1].imshow(self.var_data,extent=[0,10.0,0,10.0])
        self.robot_drawn_2, = self.ax[1].plot(self.robot_pose[0],self.robot_pose[1],'o',color='white',markersize=10)
        self.target_drawn_2, = self.ax[1].plot(self.robot_target[0],self.robot_target[1],'o',color='red',markersize=10)

        self.ax[2].set_title("Reward (unweighted by distance)")
        self.cost_uw_drawn = self.ax[2].imshow(self.cost_unweighted_data,extent=[0,10.0,0,10.0])
        self.robot_drawn_3, = self.ax[2].plot(self.robot_pose[0],self.robot_pose[1],'o',color='white',markersize=10)
        self.target_drawn_3, = self.ax[2].plot(self.robot_target[0],self.robot_target[1],'o',color='red',markersize=10)

        self.ax[3].set_title("Reward (weighted by distance)")
        self.cost_drawn = self.ax[3].imshow(self.cost_data,extent=[0,10.0,0,10.0])
        self.robot_drawn_4, = self.ax[3].plot(self.robot_pose[0],self.robot_pose[1],'o',color='white',markersize=10)
        self.target_drawn_4, = self.ax[3].plot(self.robot_target[0],self.robot_target[1],'o',color='red',markersize=10)

        self.ax[4].set_title("Ground Truth")
        self.gnd_truth_drawn = self.ax[4].imshow(self.ground_truth,extent=[0,10.0,0,10.0])

        plt.ion()
        plt.show()
        
        # Initialize Subscribers
        self.mean_sub = self.create_subscription(Float64MultiArray,"/model_mean",self.mean_callback,10)
        self.var_sub = self.create_subscription(Float64MultiArray,"/model_var",self.var_callback,10)
        self.cost_uw_sub = self.create_subscription(Float64MultiArray,"/model_cost_unweighted",self.cost_uw_callback,10)
        self.cost_sub = self.create_subscription(Float64MultiArray,"/model_cost",self.cost_callback,10)
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

    def cost_uw_callback(self,msg: Float64MultiArray):
        res_x = 100
        res_y = 100
        data = np.zeros(shape=(res_x,res_y))
        for i,val in enumerate(msg.data):
            x = i // res_x
            y = i % res_x
            data[res_y - 1 - y, x] = val
        print(f"Got unweighted reward: {data}")
        print(f"Min UW reward: {np.min(data)}")
        print(f"Max UW reward: {np.max(data)}")
        self.cost_unweighted_data = data

    def cost_callback(self,msg: Float64MultiArray):
        res_x = 100
        res_y = 100
        data = np.zeros(shape=(res_x,res_y))
        for i,val in enumerate(msg.data):
            x = i // res_x
            y = i % res_x
            data[res_y - 1 - y, x] = val
        print(f"Got weighted reward: {data}")
        print(f"Min reward: {np.min(data)}")
        print(f"Max reward: {np.max(data)}")
        self.cost_data = data
    
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

        vmin = np.min(self.cost_unweighted_data)
        vmax = np.max(self.cost_unweighted_data)
        if vmin == vmax: # We want to cover a range of color in case all our input is the same
            vmax = vmin + 1e-6
        self.cost_uw_drawn.set_clim(vmin=vmin, vmax=vmax)
        self.cost_uw_drawn.set_data(self.cost_unweighted_data)

        vmin = np.min(self.cost_data)
        vmax = np.max(self.cost_data)
        if vmin == vmax: # We want to cover a range of color in case all our input is the same
            vmax = vmin + 1e-6
        self.cost_drawn.set_clim(vmin=vmin, vmax=vmax)
        self.cost_drawn.set_data(self.cost_data)

        self.robot_drawn.set_data(self.robot_pose[0],self.robot_pose[1])
        self.target_drawn.set_data(self.robot_target[0],self.robot_target[1])
        self.robot_drawn_2.set_data(self.robot_pose[0],self.robot_pose[1])
        self.target_drawn_2.set_data(self.robot_target[0],self.robot_target[1])
        self.robot_drawn_3.set_data(self.robot_pose[0],self.robot_pose[1])
        self.target_drawn_3.set_data(self.robot_target[0],self.robot_target[1])
        self.robot_drawn_4.set_data(self.robot_pose[0],self.robot_pose[1])
        self.target_drawn_4.set_data(self.robot_target[0],self.robot_target[1])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(Visualizer())
    rclpy.shutdown()