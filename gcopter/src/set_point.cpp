#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher goal_pub;

void setgoal()
{
    // 创建一个目标点消息
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.stamp = ros::Time::now();
    
    sleep(1.0);
    goal_msg.header.frame_id = "map"; // 坐标系为地图坐标系
    goal_msg.pose.position.x = -5.385;   // 设置目标点的 x 坐标
    goal_msg.pose.position.y = -5.788;   // 设置目标点的 y 坐标
    goal_msg.pose.orientation.z = 0.0; // 默认朝向

    // 发布目标点消息
    goal_pub.publish(goal_msg);
    
    // 输出提示信息
    ROS_INFO("Published goal pose: x = %f, y = %f", goal_msg.pose.position.x, goal_msg.pose.position.y);
    
    sleep(1.0);
    goal_msg.pose.position.x = 5.385;   // 设置目标点的 x 坐标
    goal_msg.pose.position.y = 5.788;   // 设置目标点的 y 坐标
    goal_msg.pose.orientation.z = 0.968; // 默认朝向

    // 发布目标点消息
    goal_pub.publish(goal_msg);
    
    // 输出提示信息
    ROS_INFO("Published goal pose: x = %f, y = %f", goal_msg.pose.position.x, goal_msg.pose.position.y);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个发布器，用于发布目标点消息到 /move_base_simple/goal 话题
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    setgoal();

    // 延时等待
    ros::Duration(1.0).sleep();

    return 0;
}
