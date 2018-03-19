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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

//using namespace std;
using namespace Eigen;
using namespace std;
/*
class IMU {

#public:
    void _msgsCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    //Matrix3f _dcm(Matrix3f angular_v);

    float _integrate(float acc, int t);

    
private:

    float m_angular_x;
    float m_angular_y;
    float m_angular_z;

    float m_acc_x_init;
    float m_acc_y_init;
    float m_acc_z_init;

    float m_acc_x;
    float m_acc_y;
    float m_acc_z;
};
*/
    
/* define variables used in this file */
const float delta_t = 0.005;
int m_count = 0;
   
// for reading bag information 
float m_angular_x;
float m_angular_y;
float m_angular_z;

float m_acc_x_init;
float m_acc_y_init;
float m_acc_z_init;

float m_acc_x;
float m_acc_y;
float m_acc_z;

// for calculating the pose and position
Matrix3f pose = MatrixXf::Identity(3,3);
Vector3f sg, vg, gg;

// for subscriber and publisher
ros::Subscriber m_sub;
ros::Publisher m_pub;
visualization_msgs::Marker marker;


Matrix3f _calPose() {
    Matrix3f B;
    B << 0, -m_angular_z*delta_t, m_angular_y*delta_t,
         m_angular_z*delta_t, 0, -m_angular_x*delta_t,
         -m_angular_y*delta_t, m_angular_x*delta_t, 0;
   
    float sigma = delta_t*sqrt(m_angular_x*m_angular_x+
                               m_angular_y*m_angular_y+
                               m_angular_z*m_angular_z);
    //cout << "sigma: " << sigma << endl;

    pose = pose*(MatrixXf::Identity(3,3)+sin(sigma)/sigma*B+((1.0-cos(sigma))/(sigma*sigma)*B*B));    
    return pose;
}

Vector3f _calPosition() {
    Vector3f ag;
    ag << m_acc_x, m_acc_y, m_acc_z;

    vg = vg + delta_t * (ag - gg);
    sg = sg + delta_t * vg;

    //cout << "sg[1]: " << sg[1] << endl;

    cout << "sg: " << sg << endl;
    return sg;
}

void _publish() {

    // write messages
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = sg[0];
    marker.pose.position.y = sg[1];
    marker.pose.position.z = sg[2];
    // scale
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    // color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    m_pub.publish(marker);
    return;
}

void _msgsCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // read initial data
    if(m_count == 0) {
        m_acc_x_init = msg->linear_acceleration.x;
        m_acc_y_init = msg->linear_acceleration.y;
        m_acc_z_init = msg->linear_acceleration.z;
        gg << m_acc_x_init, m_acc_y_init, m_acc_z_init;
        m_count++;
    }

    m_angular_x = msg->angular_velocity.x;
    m_angular_y = msg->angular_velocity.y;
    m_angular_z = msg->angular_velocity.z;

    m_acc_x = msg->linear_acceleration.x - m_acc_x_init;
    m_acc_y = msg->linear_acceleration.y - m_acc_y_init;
    m_acc_z = msg->linear_acceleration.z - m_acc_z_init;

    // calculate pose and position
    _calPose();
    _calPosition();

    _publish();

    return;
}

int main(int argc, char **argv) {

    // initialize the subscriber and publisher
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    
    // for subscribing rosbag information
    m_sub = n.subscribe("/imu/data", 1000, _msgsCallback);
    
    m_pub = n.advertise<visualization_msgs::Marker>("marker", 10);
    
    ros::Rate r(10);
    /*
    while(ros::ok()) {
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();

        marker.ns = "basic_shapes";
        marker.id = 0;
 
        marker.type = visualization_msgs::Marker::SPHERE;

        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 
    }
    */
    ros::spin();

    return 0;
}
