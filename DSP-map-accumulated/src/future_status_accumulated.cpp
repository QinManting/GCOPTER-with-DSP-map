#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "dsp_dynamic.h"

ros::Subscriber future_status_sub;
ros::Publisher accumulated_status_pub;
pcl::PointCloud<pcl::PointXYZRGB> future_status_cloud;
DSPMap my_map;

// 将获取的ros话题信息转换成PointXYZRGB类型的点云信息
void futrureStatusCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    
    pcl::fromPCLPointCloud2(pcl_pc2, future_status_cloud);
}

// 计算当前点所在的体素索引
// int calculateVoxelIndex(pcl::PointXYZRGB &current_point)
// {

// }

// 计算同一体素不同预测时间的点的叠加权重
float calculateCombinedWeight(int voxel_index, pcl::PointCloud<pcl::PointXYZRGB> future_status_cloud)
{

}


void accumulate()
{
    pcl::PointCloud<pcl::PointXYZRGB> combined_future_status_cloud;
    int voxel_index;

    // 初始化 combined_future_status_cloud
    combined_future_status_cloud.points.resize(future_status_cloud.size());

    // 遍历 future_status_cloud 中的每个点
    for(size_t i = 0; i < future_status_cloud.points.size(); ++i) {
        // 获取当前点
        pcl::PointXYZRGB& current_point = future_status_cloud.points[i];

        my_map.getPointVoxelsIndexPublic(current_point.x, current_point.y,current_point.z,voxel_index);
 
        // int voxel_index = calculateVoxelIndex(current_point);
 
        float combined_weight = calculateCombinedWeight(voxel_index, future_status_cloud);
 
        // updatePointColor(current_point, combined_weight);

        // 将更新后的点添加到 combined_future_status_cloud 中
        combined_future_status_cloud.points[i] = current_point;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "future status accumulated");

    ros::NodeHandle nh;

    future_status_sub = nh.subscribe("/my_map/future_status", 1, futrureStatusCallback);

    accumulated_status_pub = nh.advertise<sensor_msgs::PointCloud2>("/my_map/accumulated_future_status", 1, true);

    accumulate();

    return 0;
}