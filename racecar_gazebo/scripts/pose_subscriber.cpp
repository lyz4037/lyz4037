/**
 * 该例程将订阅/cmd_vel话题，消息类型geometry_msgs/Twist
 */
#include <ros/ros.h>
#include <geometry_msgs/Twish.h>

// 接收到订阅的消息后，会进入消息回调函数
void poseCallback(const geometry_msgs::Twist& cmd_vel)
{
	//将接收到的消息打印出来
	ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
	ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);

}

int main(int argc,char **argv)
{
	//初始化ROS节点
	ros::init(argc,argv,"cmd_vel_listener");
	
	//创建节点句柄
	ros::NodeHandle n;

	//创建一个Subscriber,订阅名为/cmd_vel的topic,注册回调函数poseCallback
	ros::Subscriber pose_sub = n.subscribe("/turtle1/pose",10,poseCallback);

	//循环等待回调函数
	ros::spin();

	return 0;
}
