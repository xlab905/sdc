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

#include <tf/transform_broadcaster.h>

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
PointCloud::Ptr cloud_final (new PointCloud);


// ros publisher
ros::Publisher m_pub_scene, m_pub_map, m_pub_final;


int _load_pcd() {

    if(pcl::io::loadPCDFile<pcl::PointXYZ> (scene_filename, *cloud_scene) == -1) {
        return 0;
    }
    else {
        cout << COUT_PREFIX << "Load scene success" << endl;
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZ> (map_filename, *cloud_map) == -1) {
        return 0;
    }
    else {
        cout << COUT_PREFIX << "Load map success" << endl;
    }
}

void _icp() {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    cout << COUT_PREFIX << "Setting source and target cloud" << endl;
    icp.setInputSource(cloud_scene);
    icp.setInputTarget(cloud_map);

     // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-20);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1e-20);

    cout << COUT_PREFIX << "Start performing icp ...";
    icp.align(*cloud_final);
    cout << "... done" << endl;
    cout << COUT_PREFIX << "has converged: " << icp.hasConverged() << endl;
    cout << COUT_PREFIX << "score: " << icp.getFitnessScore() << endl;
    cout << COUT_PREFIX << "matrix: " << endl;
    cout << icp.getFinalTransformation() << endl;
}

int main(int argc, char** argv) {
    if(argc != 3) {
        cout << COUT_PREFIX;
        cout << "Usage: ";
        cout << "rosrun 0560032_hw4 hw4_node ";
        cout << "<source cloud> <target cloud>" << endl;
        return -1;
    }

    scene_filename = argv[1];
    map_filename = argv[2];

    // initialize ros
    ros::init(argc, argv, "pcl");
    ros::NodeHandle n;

    // for publishing pcl information
    m_pub_scene = n.advertise<sensor_msgs::PointCloud2> ("output_scene", 1);
    m_pub_map = n.advertise<sensor_msgs::PointCloud2> ("output_map", 1);
    m_pub_final = n.advertise<sensor_msgs::PointCloud2> ("output_final", 1);

    if(!_load_pcd()) {return -1;};
    _icp();

    // publish point cloud
    cloud_scene->header.frame_id = "/scene";
    cloud_map->header.frame_id = "/map";
    cloud_final->header.frame_id = "/scene";

    ros::Rate loop_rate(4);

    while(n.ok()) {
        pcl_conversions::toPCL(ros::Time::now(), cloud_scene->header.stamp);
        pcl_conversions::toPCL(ros::Time::now(), cloud_map->header.stamp);
        pcl_conversions::toPCL(ros::Time::now(), cloud_final->header.stamp);
        m_pub_scene.publish(cloud_scene);
        m_pub_map.publish(cloud_map);
        m_pub_final.publish(cloud_final);
        
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "scene"));



        ros::spinOnce();
        loop_rate.sleep();
    }


    //ros::spin();

    return 0;
}
