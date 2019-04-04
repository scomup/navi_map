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
#include "MultiLevelGrid.h"



pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>());

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "info_marker_publisher1");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/map3d", 1, cloud_cb);

    std::cout << "wait map3d topic\n";
    while (cloud == nullptr && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ros::spinOnce();
    }

    visualization_msgs::MarkerArray marker_array;

    GlobalPlan::MultiLevelGrid mlg(0.8, 0.1);
    mlg.setInput(cloud);
    auto traversable_node = mlg.traversable_node();
    for (auto& n : traversable_node)
    {
        createMarker(n, 1, marker_array);
    }

    auto obstacle_node = mlg.obstacle_node();
    for (auto& n : obstacle_node)
    {
        createMarker(n, 0, marker_array);
    }

    auto cellmap3d = mlg.cellmap3d();
    for (auto& m : cellmap3d)
    {
        auto& cell = m.second;
        cloud_map->push_back(pcl::PointXYZ(cell->position(0), cell->position(1), cell->position(2)));
    }

    /*
    GlobalPlan::NDTVoxel<pcl::PointXYZ> voxel; //(voxel_ids, cloud);
    voxel.setInput(cloud);
    auto nodes = voxel.getAllNode();

    visualization_msgs::MarkerArray marker_array;
    for (auto nn : nodes)
    {
        //if(cont == 500)
        //    break;

        bool state = createMarker(nn, cloud, marker_array);

        if(!state)
            continue;


        double min_x,min_y,min_z,max_x,max_y,max_z;
        voxel.getNodeBoundary(nn,min_x,min_y,min_z,max_x,max_y,max_z);
        auto centroid = nn->centroid;
        auto covariance = nn->covariance;

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R(svd.matrixU());
        R.col(0).normalize();
        R.col(1).normalize();
        R.col(2) = R.col(0).cross(R.col(1));
        R.col(2).normalize();
        R.col(0) = R.col(1).cross(R.col(2));
        R.col(0).normalize();
        Eigen::Matrix3d S;
        S << sqrt(svd.singularValues().x()), 0, 0,
            0, sqrt(svd.singularValues().y()), 0,
            0, 0, sqrt(svd.singularValues().z());

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R * S;
        T(0, 3) = centroid(0);
        T(1, 3) = centroid(1);
        T(2, 3) = centroid(2);
        Eigen::Matrix4d Tinv = T.inverse();

        for (double x = min_x; x < max_x; x += 0.1)
        {
            for (double y = min_y; y < max_y; y += 0.1)
            {
                double z = GlobalPlan::GetZOnSurface<double>(Tinv, x, y, 6);
                cloud_map->push_back(pcl::PointXYZ(x,y,z));
            }
        }
    }
    */

    /*
    std::cout<<"try ransac!\n"<<std::endl;
    for (auto n : nodes)
    {
        std::cout<<n->point_idx.size()<<std::endl;
        if(n->point_idx.size() < 100)
            continue;
        auto plane = GlobalPlan::RansacPlane<pcl::PointXYZ>(cloud, n->point_idx);

        for (auto i : n->point_idx)
        {
            double x = cloud->points[i].x;
            double y = cloud->points[i].y;
            double a = plane[0];
            double b = plane[1];
            double c = plane[2];
            double d = plane[3];
            double z = -(a * x + b * y + d) / c;
            auto p = pcl::PointXYZ(x, y, z);
            cloud_map->push_back(p);
        }
    }*/

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_map, output);
    output.header.frame_id = "map";

    auto cell_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 100, true);
    cell_pub.publish(output);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1, true);
    marker_pub.publish(marker_array);

    ros::spin();

    /*
    std::vector<Eigen::Vector3i> voxel_ids(cloud->points.size());

    for (int i = 0; i < (int)cloud->points.size(); i++)
    {
        Eigen::Vector3i &vid = voxel_ids[i];
        pcl::PointXYZ p = cloud->points[i];

        vid(0) = static_cast<int>(floor(p.x / 0.1));
        vid(1) = static_cast<int>(floor(p.y / 0.1));
        vid(2) = static_cast<int>(floor(p.z / 0.1));
    }
    std::cout<<"1.."<<std::endl;

    GlobalPlan::NDTVoxel<pcl::PointXYZ> voxel;//(voxel_ids, cloud);
    voxel.setInput(voxel_ids, cloud);
    std::cout<<"OK.."<<std::endl;

    auto nodes = voxel.getNDTVoxelLevel(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>());

    for (auto n : nodes)
    {
        cloud_map->push_back(pcl::PointXYZ(n.centroid(0), n.centroid(1), n.centroid(2)));
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_map, output);
    output.header.frame_id = "map";

    auto cell_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 100, true);
    cell_pub.publish(output);
        ros::spin();
        */

    //GlobalPlan::NaviMapVisualization map_visual(&navi_map);

    /*
    auto s = navi_map.FindNearestCell(Eigen::Vector3d(0,-1.5,0));
    auto g = navi_map.FindNearestCell(Eigen::Vector3d(0,15,3));
    std::vector<Eigen::Vector3d> path;
    navi_map.FindPath(s, g, path);
    ros::Publisher plan_pub;
    plan_pub = nh.advertise<nav_msgs::Path>("global_plan", 1, true);
    nav_msgs::Path path_msg;

    path_msg.poses.resize(path.size());
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();


    //graph_->FindPath(start_idx, goal_idx, std::vector<uint> path);
    int i = 0;
    for (Eigen::Vector3d p : path)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z()+0.3;
        path_msg.poses[i++] = pose;
    }
    plan_pub.publish(path_msg);

    auto cell_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 100, true);
    cell_pub.publish(map_visual.createCellMarker());

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1, true);
    marker_pub.publish(map_visual.createCovarianceMarker());

    std::cout<<"OK!"<<std::endl;

    ros::spin();
*/
    return 0;
}