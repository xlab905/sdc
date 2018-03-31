#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace Eigen;
using namespace std;

//// Variables used in this file ////

// point cloud data
string scene_filename, map_filename;
PointCloud::Ptr cloud_scene (new PointCloud);
PointCloud::Ptr cloud_map (new PointCloud);


// ros publisher
ros::Publisher m_pub_scene, m_pub_map;

void _load_pcd() {
    scene_filename = "/home/kevin/catkin_ws/src/scene.pcd";
    map_filename = "/home/kevin/catkin_ws/src/map.pcd";

    if(pcl::io::loadPCDFile<pcl::PointXYZ> (scene_filename, *cloud_scene) == -1) {
        PCL_ERROR("Could not real file %s", scene_filename);
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (map_filename, *cloud_map) == -1) {
        PCL_ERROR("Could not real file %s", map_filename);
    }
    cout << "Load PCDs success" << endl;
}

void _icp() {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //icp.setInputSource(cloud_scene);
    //icp.setInputTarget(cloud_map);
    //PointCloud cloud_final;
    //icp.align(cloud_final);
    //cout << "Perform ICP done" << endl;
}

int main(int argc, char** argv) {
    // initialize ros
    ros::init(argc, argv, "pcl");
    ros::NodeHandle n;

    // for publishing pcl information
    m_pub_scene = n.advertise<PointCloud> ("output_scene", 1);
    m_pub_map = n.advertise<PointCloud> ("output_map", 1);

    _load_pcd();
    _icp();

    // publish point cloud
    cloud_scene->header.frame_id = "/map";
    cloud_map->header.frame_id = "/map";

    ros::Rate loop_rate(4);

    while(n.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), cloud_scene->header.stamp);
        pcl_conversions::toPCL(ros::Time::now(), cloud_scene->header.stamp);
        m_pub_scene.publish(cloud_scene);
        m_pub_map.publish(cloud_map);
        ros::spinOnce();
        loop_rate.sleep();
    }


    //ros::spin();

    return 0;
}

