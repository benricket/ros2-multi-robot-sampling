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

        self.declare_parameter("num_robots",2)
        self.num_robots = self.get_parameter("num_robots").get_parameter_value().integer_value

        self.robot_poses = [[0.0,0.0] for i in range(self.num_robots)]
        self.robot_targets = [[0.0,0.0] for i in range(self.num_robots)]

        # Determine how many colors we need to represent our robots
        cmap = plt.get_cmap("jet")
        colors = []
        for i in range(self.num_robots):
            colors.append(cmap(i / self.num_robots))

        self.num_plots_with_robots = 4
        robot_markersize = 10
        target_markersize = 5
        self.timer = self.create_timer(0.01,self.timer_callback)
        self.fig, self.ax = plt.subplots(1,5)

        self.robots_drawn = []
        self.targets_drawn = []

        for i in range(self.num_plots_with_robots):
            robot_on_ax = []
            target_on_ax = []
            for robot_id in range(self.num_robots):
                color = colors[robot_id]
                r_drawn, = self.ax[i].plot(self.robot_poses[robot_id][0],self.robot_poses[robot_id][1],'o',color=color,markersize=robot_markersize)
                t_drawn, = self.ax[i].plot(self.robot_targets[robot_id][0],self.robot_targets[robot_id][1],'o',color=color,markersize=target_markersize)
                robot_on_ax.append(r_drawn)
                target_on_ax.append(t_drawn)
            self.robots_drawn.append(robot_on_ax)
            self.targets_drawn.append(target_on_ax)

        self.ax[0].set_title("Predicted Values")
        self.mean_drawn = self.ax[0].imshow(self.mean_data,extent=[0,10.0,0,10.0])
        
        self.ax[1].set_title("Prediction Variance")
        self.var_drawn = self.ax[1].imshow(self.var_data,extent=[0,10.0,0,10.0])
        
        self.ax[2].set_title("Reward (unweighted by distance)")
        self.cost_uw_drawn = self.ax[2].imshow(self.cost_unweighted_data,extent=[0,10.0,0,10.0])
        
        self.ax[3].set_title("Reward (weighted by distance)")
        self.cost_drawn = self.ax[3].imshow(self.cost_data,extent=[0,10.0,0,10.0])
        
        self.ax[4].set_title("Ground Truth")
        self.gnd_truth_drawn = self.ax[4].imshow(self.ground_truth,extent=[0,10.0,0,10.0])

        plt.ion()
        plt.show()
        
        # Initialize Subscribers
        self.mean_sub = self.create_subscription(Float64MultiArray,"/model_mean",self.mean_callback,10)
        self.var_sub = self.create_subscription(Float64MultiArray,"/model_var",self.var_callback,10)
        self.cost_uw_sub = self.create_subscription(Float64MultiArray,"/model_cost_unweighted",self.cost_uw_callback,10)
        self.cost_sub = self.create_subscription(Float64MultiArray,"/model_cost",self.cost_callback,10)
        self.robot_pose_subs = []
        self.robot_target_subs = []

        for i in range(self.num_robots):
            self.robot_pose_subs.append(self.create_subscription(PoseStamped,f"/robot{i}/pos",lambda msg,robot_id=i: self.robot_pose_callback(msg,robot_id),10))
            self.robot_target_subs.append(self.create_subscription(PoseStamped,f"/robot{i}/target",lambda msg,robot_id=i: self.robot_target_callback(msg,robot_id),10))

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
    
    def robot_pose_callback(self,msg: PoseStamped, id: int):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_poses[id] = [x,y]

    def robot_target_callback(self,msg: PoseStamped, id: int):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_targets[id] = [x,y]
        
    def timer_callback(self):
        print(f"timer callback: poses are {self.robot_poses}")

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

        for ax_id in range(self.num_plots_with_robots):
            for robot_id in range(self.num_robots):
                self.robots_drawn[ax_id][robot_id].set_data(self.robot_poses[robot_id][0],self.robot_poses[robot_id][1])
                self.targets_drawn[ax_id][robot_id].set_data(self.robot_targets[robot_id][0],self.robot_targets[robot_id][1])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(Visualizer())
    rclpy.shutdown()