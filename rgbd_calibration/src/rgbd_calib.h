#ifndef RGBD_CALIB_H
#define RGB_CALIB_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "rgbd_calibration/RgbdMsg.h"


class RgbdCalib
{
public:
    RgbdCalib();
    ~RgbdCalib();

private:
    void pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);
    bool pcl_ransac_method(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double fitparam[4]);

private:
    ros::Publisher m_pubPoints;
    ros::Publisher m_pubFilterPot;
    ros::Publisher m_pubTest;
    ros::Publisher m_pubPose;
    ros::Publisher m_pubCalibrationRes;
    ros::Subscriber sub_pointcloud__;
    double origin_x_;
    double camera_z_;
    double camera_roll_;
    double camera_pitch_;
    double sac_threshold_;
    int sac_iteration_;
    double cali_z_diff_;
    double cali_angle_diff_;
    int roi_w_rate_;
    int roi_h_rate_;
    int used_method_;

    ros::NodeHandle *nh;
};

#endif
