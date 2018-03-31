#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/program_options.hpp>

#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

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

#define COUT_PREFIX "\033[1;33m" << "[HW4] " << "\033[0m"

//// Variables used in this file ////

// point cloud data
string scene_filename, map_filename;
PointCloud::Ptr cloud_scene (new PointCloud);
PointCloud::Ptr cloud_map (new PointCloud);


// ros publisher
ros::Publisher m_pub_scene, m_pub_map;

void _load_pcd() {
    scene_filename = "/home/sdc/catkin_ws/src/scene.pcd";
    map_filename = "/home/sdc/catkin_ws/src/map.pcd";

    if(pcl::io::loadPCDFile<pcl::PointXYZ> (scene_filename, *cloud_scene) == -1) {
        PCL_ERROR("Could not real file %s", scene_filename);
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (map_filename, *cloud_map) == -1) {
        PCL_ERROR("Could not real file %s", map_filename);
    }
    cout << COUT_PREFIX << "Load PCDs success" << endl;
}

void _icp() {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    cout << COUT_PREFIX << "Setting source and target cloud" << endl;
    icp.setInputSource(cloud_scene);
    icp.setInputTarget(cloud_map);
    cout << COUT_PREFIX << "Start performing icp ...";
    PointCloud cloud_final;
    icp.align(cloud_final);
    cout << "... done" << endl;
    cout << COUT_PREFIX << "has converged: " << icp.hasConverged() << endl;
    cout << COUT_PREFIX << "score: " << icp.getFitnessScore() << endl;
    cout << COUT_PREFIX << "matrix: " << endl;
    cout << icp.getFinalTransformation() << endl;
}

int main(int argc, char** argv) {
    // parse arguments
    namespace bpo = boost::program_options;
    bpo::options_description desc("HW4");
    desc.add_options()
        ("help,h", "print usage")
        ("source_cloud,s", "Source cloud")
        ("target_cloud,t", "Target cloud")
    ;

    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);

    if(vm.count("help")) {
        cout << COUT_PREFIX << desc << endl;
        return 0;
    }
    if(vm.count("source_cloud")) {
        scene_filename = vm["source_cloud"].as<string>();  
    }
    if(vm.count("target_cloud")) {
        map_filename = vm["target_cloud"].as<string>();
    }

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

