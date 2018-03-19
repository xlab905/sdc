#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
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


//void _msgsCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
//float _integrate(float acc, int t);
    
const float delta_t = 0.005;
int m_count = 0;
float m_angular_x;
float m_angular_y;
float m_angular_z;

float m_acc_x_init;
float m_acc_y_init;
float m_acc_z_init;

float m_acc_x;
float m_acc_y;
float m_acc_z;

Matrix3f pose = MatrixXf::Identity(3,3);
Vector3f sg, vg, gg;

Matrix3f _calPose(float angular_x, float angular_y, float angular_z) {
    
    Matrix3f B;
    B << 0, -angular_z*delta_t, angular_y*delta_t,
         angular_z*delta_t, 0, -angular_x*delta_t,
         -angular_y*delta_t, angular_x*delta_t, 0;
   
    float sigma = delta_t*sqrt(angular_x*angular_x+angular_y*angular_y+angular_z*angular_z);
    //cout << "sigma: " << sigma << endl;

    //cout << "2: " << sin(sigma)/sigma*B << endl;
    //cout << "3: " << (1.0-cos(sigma))/

    pose = pose*(MatrixXf::Identity(3,3)+sin(sigma)/sigma*B+((1.0-cos(sigma))/(sigma*sigma)*B*B));    

    cout << "pose: " << pose << endl;

    return pose;
}

Vector3f _calPosition(float acc_x, float acc_y, float acc_z) {

    Vector3f ag;
    ag << acc_x, acc_y, acc_z;

    vg = vg + delta_t * (ag - gg);
    sg = sg + delta_t * vg;

    cout << "sg: " << sg << endl;
    return sg;
}

void _msgsCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    //cout << "In msgs callback" << endl; 
    // read data
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

    _calPose(m_angular_x, m_angular_y, m_angular_z);
    _calPosition(m_acc_x, m_acc_y, m_acc_z);

    return;
    //ROS_INFO("Imu time: [%d.%d]", msg->header.stamp.sec, msg->header.stamp.nsec);
    //ROS_INFO("angular: %f", msg->angular_velocity.x);
    //ROS_INFO("z acc: %f", m_acc_z);
}

//Matrix3f _dcm(Matrix3f angular_v) {
    
//}

int main(int argc, char **argv) {

    // initialize the publisher
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/imu/data", 1000, _msgsCallback);
    
    ros::spin();

    return 0;
}
