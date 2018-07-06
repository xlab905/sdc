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
#include <pcl/common/common.h>
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
// ros publisher and subscriber
ros::Subscriber m_sub_pcd, m_sub_visual;
ros::Publisher m_pub_visual;

visualization_msgs::Marker marker;

int m_id = 0, m_max_range = 0;
int m_count = 0;
vector<float> pos_x, pos_y;


void _visualCallback(const visualization_msgs::Marker& msg) {
    //cout << "points: " << msg.pose.position << endl;
    
    // setting message
    /*
    marker.ns = msg.ns;
    marker.id = msg.id;
    marker.pose.position.x = msg.pose.position.x;
    marker.pose.position.y = msg.pose.position.y;
    marker.pose.position.z = msg.pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = msg.scale.x;
    marker.scale.y = msg.scale.y;
    marker.scale.z = msg.scale.z;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.lifetime = msg.lifetime;
    m_pub_visual.publish(marker);
    */
    //cout << "id: " << msg.id << endl;
    // calculation
    int x, y, dist;
    x = msg.pose.position.x;
    y = msg.pose.position.y;
    if(msg.id == 0) {
        pos_x.push_back(x);
        pos_y.push_back(y);
    }
    /*
    dist = x*x+y*y;
    if(dist > m_max_range) { m_max_range = dist; }
    if(marker.id>m_id) { m_id = marker.id; }
    cout << "m_id: " << m_id << endl;
    cout << "m_max_range: " << m_max_range << endl; 
    */
    return;
}

void _pcdCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    cout << "point cloud row step = " << msg->row_step << endl;
    cout << "point cloud height   = " << msg->height << endl;
    cout << "point cloud size     = " << msg->row_step*msg->height << endl;

    return;
}

int main(int argc, char** argv) {

    // initialize ros
    ros::init(argc, argv, "pcl");
    ros::NodeHandle n;
    
    marker.header.frame_id = "/esr_can0_frame";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    m_pub_visual = n.advertise<visualization_msgs::Marker>("rec", 0);

    // subscribe
    //m_sub_pcd = n.subscribe("/points_raw", 1000, _pcdCallback);
    m_sub_visual = n.subscribe("esr_can0_visualization", 1000, _visualCallback);

    ros::Rate loop_rate(4);
    ros::spin();

    return 0;
}
