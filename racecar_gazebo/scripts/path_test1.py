#!/usr/bin/env python

import rospy
from sensor_msgs/LaserScan import ranges
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

class following_path:
    def __init__(self):
        self.current_pose = rospy.Subscriber('/pf/pose/odom', Odometry, self.callback_read_current_position, queue_size=1)
        self.Pose = []
        self.path_pose = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.callback_read_path, queue_size=1)
        self.path_info = []
        self.Goal = []
        self.navigation_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.reach_goal = False
        self.MAX_VELOCITY = 0.8
        self.MIN_VELOCITY = 0
        self.max_angle = 1
        self.steering_velocity = 1
        self.jerk = 0.0
        self.acceleration = 0.0
        self.LOOKAHEAD_DISTANCE = 0.30
        self.Low_Speed_Mode = False
        
    def callback_read_path(self, data):
        #组织姿势信息，只要求（x，y）和方向
        #读取实时姿势信息并将其加载到路径信息中
        self.path_info = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            self.path_info.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal = list(self.path_info[-1]) # 将全局路径的最后一个姿势设置为目标位置（我在这里用的局部路径）

    def callback_read_current_position(self, data):
        if self.reach_goal: # 停止更新信息。
            self.path_info = []
            self.Pose = []
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0

        if not len(self.path_info) == 0:
            #读取路径信息到路径点列表
            path_points_x = [float(point[0]) for point in self.path_info]
            path_points_y = [float(point[1]) for point in self.path_info]
            path_points_w = [float(point[2]) for point in self.path_info]

            #从粒子过滤器读取汽车的当前姿势（其实也就是从里程计里面读取当前姿势）  
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w

            # 将四元数角度转换为欧拉角度
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.Pose = [float(x), float(y), float(yaw)]

            if self.dist(self.Goal, self.Pose) < 1.0:
                self.Low_Speed_Mode = True
                if self.dist(self.Goal, self.Pose) < 0.1:
                    self.reach_goal = True
                    print('Goal Reached!')
                else:
                    print('Low Speed Mode ON!')
            else:
                self.Low_Speed_Mode = False

            # 2. 找到距离车辆最近的路径点，距离车辆当前位置的前方距离大于等于1（这个我改了但是不知道改后是多少）
            dist_array = np.zeros(len(path_points_x))

            for i in range(len(path_points_x)):
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]), (x,y))
            
            goal = np.argmin(dist_array) #首先假设最接近的点是目标点 
            goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 0.15)) & (dist_array > (self.LOOKAHEAD_DISTANCE - 0.15)))[0]
            for id in goal_array:
                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
                if abs(diff_angle) < np.pi/4: # 检查最靠近前方方向的那个
                    goal = id
                    break

            L = dist_array[goal]
            # 3.将目标点转换为车辆坐标。
            glob_x = path_points_x[goal] - x 
            glob_y = path_points_y[goal] - y 
            goal_x_veh_coord = glob_x*np.cos(yaw) + glob_y*np.sin(yaw)
            goal_y_veh_coord = glob_y*np.cos(yaw) - glob_x*np.sin(yaw)

            # 4. 计算曲率=2x1/2
            # 通过车载控制器将曲率转化为方向盘转角。
            # 提示：您可能需要翻到负值，因为对于VESC来说，右转向角是负值。
            #diff_angle = path_points_w[goal] - yaw #找到转弯角度
	    sum_left=0
	    sum_right=0
	    for i in range(90,180,1)
		sum_right += ranges[i]
	    for i in range(180,270,1)
		sum_left += ranges[i]
	    diff_angle = 2 * (sum_left - sum_right) / max(sum_left,sum_right)
            r = L/(2*math.sin(diff_angle)) # 计算转弯半径
            angle = 2 * math.atan(0.4/r) # 找出车轮转弯半径
            angle = np.clip(angle, -self.max_angle, self.max_angle) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
            angle = (0 if abs(angle) < 0.1 else angle)
            VELOCITY = self.speed_control(angle, self.MIN_VELOCITY, self.MAX_VELOCITY)

            #将速度和角度数据写入ackermann消息
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = VELOCITY
            ackermann_control.drive.steering_angle = angle
            ackermann_control.drive.steering_angle_velocity = self.steering_velocity   
        else:
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
        
        self.navigation_input.publish(ackermann_control)
    
    # 计算两个二维点p1和p2之间的欧几里德距离（就是两点之间的距离）
    def dist(self, p1, p2):
	try:
		return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
	except:
		return 0.5

    # 计算轿厢方向与目标方向的夹角
    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)
        sin_ang = LA.norm(np.cross(v1, v2))
        return np.arctan2(sin_ang, cos_ang) 

    # 把车速控制在限速范围内
    def speed_control(self, angle, MIN_VELOCITY, MAX_VELOCITY):
        #假设速度与偏航角成线性关系 
        if self.Low_Speed_Mode:
            Velocity = 0.7
        else:
            k = (MIN_VELOCITY - MAX_VELOCITY)/self.max_angle + 0.5
            Velocity = k * abs(angle) + MAX_VELOCITY
        return Velocity

    
if __name__ == "__main__":

    rospy.init_node("pursuit_path")
    following_path()
    rospy.spin()
