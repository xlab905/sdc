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
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

//using namespace std;
using namespace Eigen;
using namespace std;
    
//// define variables used in this file ////
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
Vector3f sg = Vector3f::Zero(), vg = Vector3f::Zero(), gg;
Vector4f q = Vector4f::Zero();


// for subscriber and publisher
ros::Subscriber m_sub;
ros::Publisher m_pub;
visualization_msgs::Marker points, line_strip;

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

    // transform to quaternion
    q[0] = sqrt(1 + pose(0,0) + pose(1,1) + pose(2,2)) / 2;
    q[1] = (pose(2,1) - pose(1,2)) / (4 * q[0]);
    q[2] = (pose(0,2) - pose(2,0)) / (4 * q[0]);
    q[3] = (pose(1,0) - pose(0,1)) / (4 * q[0]);
    
    return pose;
}

Vector3f _calPosition() {
    Vector3f ag, sg_old;
    ag << m_acc_x, m_acc_y, m_acc_z;

    //vg = vg + delta_t * (ag - gg);
    vg = vg + delta_t * (ag);
    sg_old = sg;
    sg = sg + delta_t * vg;

    //sg = sg + pose * (sg - sg_old);
    cout << "sg: " << sg << endl;
    return sg;
}

void _publish() {
    // write messages
    points.header.frame_id = line_strip.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();

    points.id = 0;
    line_strip.id = 1;
    
    points.type = visualization_msgs::Marker::SPHERE;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    points.action = line_strip.action = visualization_msgs::Marker::ADD;

    // scale
    points.scale.x = 0;
    points.scale.y = 0;
    points.scale.z = 0;
    line_strip.scale.x = 0.1;
    
    // color
    points.color.r = 0.0f;
    points.color.g = 1.0f;
    points.color.b = 0.0f;
    points.color.a = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    // quaternion
    points.pose.orientation.w = q[0];
    points.pose.orientation.x = q[1];
    points.pose.orientation.y = q[2];
    points.pose.orientation.z = q[3];

    // position
    geometry_msgs::Point p;
    p.x = sg[0];
    p.y = sg[1];
    p.z = sg[2];
    points.points.push_back(p);
    line_strip.points.push_back(p);

    //publish
    m_pub.publish(points);
    m_pub.publish(line_strip);
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
    
    ros::Rate r(30);
    ros::spin();

    return 0;
}
