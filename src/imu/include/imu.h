#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>

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

using namespace Eigen;
using namespace std;

class IMU {

public:
    IMU();
    ~IMU();

    int Load(int argc, char **argv);

    void _msgsCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    void _publish();

    Matrix3f _calPose();

    Vector3f _calPosition();
    
private:
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

    Matrix3f pose = Matrix3f::Identity(3,3);
    Vector3f sg, vg, gg;

    ros::Subscriber m_sub;
    ros::Publisher m_pub;
    visualization_msgs::Marker marker;
};
