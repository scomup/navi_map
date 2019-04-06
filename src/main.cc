#include <chrono>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <unordered_map>
#include <utility>

#include "NDTVoxel.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include "NaviMap.h"



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
GlobalPlan::NaviMap* navi_map;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // ... do data processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *local_cloud);
    size_t j = 0;
    for (size_t i = 0; i < local_cloud->points.size(); ++i)
    {
        if (!pcl_isfinite(local_cloud->points[i].x) ||
            !pcl_isfinite(local_cloud->points[i].y) ||
            !pcl_isfinite(local_cloud->points[i].z))
            continue;
        local_cloud->points[j] = local_cloud->points[i];
        j++;
    }
    if (j != local_cloud->points.size())
    {
        // Resize to the correct size
        local_cloud->points.resize(j);
    }
    cloud = local_cloud;
}



bool createMarker(GlobalPlan::NDTVoxelNode *node, int rgb,
                  visualization_msgs::MarkerArray &marker_array)
{


    auto centroid = node->centroid;
    auto covariance = node->covariance;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3d R(svd.matrixU());
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2) = R.col(0).cross(R.col(1));
    R.col(2).normalize();
    R.col(0) = R.col(1).cross(R.col(2));
    R.col(0).normalize();
    Eigen::Quaterniond q(R);

    static int id = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes" + std::to_string(id);
    marker.id = id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = sqrt(svd.singularValues().x()) * 5;
    marker.scale.y = sqrt(svd.singularValues().y()) * 5;
    marker.scale.z = sqrt(svd.singularValues().z()) * 5;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    if (rgb == 0)
    {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.3;
    }
    else if(rgb == 1)
    {
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        marker.color.a = 0.3;
    }
    else
    {
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 0.3;
    }

    marker_array.markers.push_back(marker);
    return true;
}
ros::Publisher plan_pub;

void goalCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    Eigen::Vector3d goal(x, y, z);

    auto s = navi_map->FindNearestCell(Eigen::Vector3d(0, -1.5, 0));
    auto g = navi_map->FindNearestCell(goal);

    std::vector<Eigen::Vector3d> path;

    navi_map->FindPath(s, g, path);
    
    nav_msgs::Path path_msg;

    path_msg.poses.resize(path.size());
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    int i = 0;
    for (Eigen::Vector3d p : path)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z() + 0.3;
        path_msg.poses[i++] = pose;
    }
    plan_pub.publish(path_msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_marker_publisher1");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map3d", 1, cloud_cb);
    plan_pub = nh.advertise<nav_msgs::Path>("global_plan", 1, true);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZI>());

    std::cout << "Wait map3d topic..\n";
    while (cloud == nullptr && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ros::spinOnce();
    }
    ros::Subscriber goal_sub_ = nh.subscribe("goal", 1, goalCB);


    visualization_msgs::MarkerArray marker_array;

    navi_map = new GlobalPlan::NaviMap(0.8, 0.1);
    navi_map->setInput(cloud);
    auto traversable_node = navi_map->traversable_node();
    for (auto& n : traversable_node)
    {
        createMarker(n, 1, marker_array);
    }

    auto obstacle_node = navi_map->obstacle_node();
    for (auto& n : obstacle_node)
    {
        createMarker(n, 0, marker_array);
    }

    auto cellmap3d = navi_map->cellmap3d();
    
    for (auto& m : cellmap3d)
    {
        auto& cell = m.second;
        pcl::PointXYZI p;
        p.x = cell->position(0);
        p.y = cell->position(1);
        p.z = cell->position(2);

        p.intensity = cell->dist;
        cloud_map->push_back(p);
    }



    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_map, output);
    output.header.frame_id = "map";

    auto cell_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 100, true);
    cell_pub.publish(output);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1, true);
    marker_pub.publish(marker_array);
    std::cout << "OK!" << std::endl;

    ros::spin();

    return 0;
}