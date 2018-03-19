#include <stdio.h>
#include <stdlib.h>
//#include <time.h>
#include <sstream>
#include <string>
#include <vector>
//#include <queue>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class IMU {

public:
    IMU();
    ~IMU();

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
