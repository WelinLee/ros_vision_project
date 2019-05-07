#include <string>
#include <math.h>
#include <time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "rgbd_calib.h"
#include "cal_tools.hpp"


const std::string point_topic = "/camera/depth/points";
const std::string frame_cam = "/camera_link";

RgbdCalib::RgbdCalib()
{
    std::string ns = ros::this_node::getName();
    nh = new ros::NodeHandle(ns);

    ros::NodeHandle node;
    origin_x_ = 0.26; camera_z_ = 0; camera_roll_ = 0.02; camera_pitch_ = 0.50;
    roi_w_rate_ = 10; roi_h_rate_ = 10;
    cali_z_diff_ = 0.03; cali_angle_diff_ = 0.02;
    node.getParam("origin_x", origin_x_);
    node.getParam("camera_z", camera_z_);
    node.getParam("camera_roll", camera_roll_);
    node.getParam("camera_pitch", camera_pitch_);
    node.getParam("roi_w_rate", roi_w_rate_);
    node.getParam("roi_h_rate", roi_h_rate_);
    if(roi_h_rate_ > 49)
        roi_h_rate_ = 10;
    if(roi_w_rate_ > 49)
        roi_w_rate_ = 10;
    node.getParam("z_diff", cali_z_diff_);
    node.getParam("angle_diff", cali_angle_diff_);

    sac_threshold_ = 0.02; sac_iteration_ = 5000;
    node.getParam("ransac_threshold", sac_threshold_);
    node.getParam("ransac_iteration", sac_iteration_);

    used_method_ = 1;
    node.getParam("method", used_method_);        //   # 0 only filters 1 LMS 2 ransac

    sub_pointcloud__ = nh->subscribe<sensor_msgs::PointCloud2>(point_topic, 1, &RgbdCalib::pointcloudCB, this);

    m_pubPoints = nh->advertise<sensor_msgs::PointCloud>("selected_points", 1);
    m_pubFilterPot = nh->advertise<sensor_msgs::PointCloud2>("filters_points", 1);

    m_pubTest = nh->advertise<geometry_msgs::PolygonStamped>("polygon_axis", 1);
    m_pubPose = nh->advertise<geometry_msgs::PoseStamped>("camera_pose", 1);

    m_pubCalibrationRes = nh->advertise<rgbd_calibration::RgbdMsg>("calibration_result", 1);
}

RgbdCalib::~RgbdCalib()
{

}

void RgbdCalib::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    std::cout << "--------------------------- Point Cloud Data ---------------------------" << "\n";
    ROS_INFO("frame_id: %s", msg->header.frame_id.c_str());
    ROS_INFO("height: %d", msg->height);
    ROS_INFO("width: %d", msg->width);
    ROS_INFO("point step: %d", msg->point_step);
    ROS_INFO("data size: %d", msg->data.size());

    std::string frame_id = msg->header.frame_id;

    double start_time = double(clock());

    //to pcl point XYZ format
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_pcd->width = msg->width;
    pcl_pcd->height = msg->height;
    pcl_pcd->is_dense = false;
    pcl_pcd->points.resize(pcl_pcd->width * pcl_pcd->height);

    //
    std::vector<MyPointXYZ> point_buffer;
    point_buffer.clear();
    static int cnt = 0;
    for (unsigned int i=0; i<msg->data.size(); i++)
    {
        if(0 == i%(msg->point_step))
        {
            unsigned char arr1[4] = {msg->data[i], msg->data[i+1], msg->data[i+2], msg->data[i+3]};
            float x = *(float *)arr1;
            unsigned char arr2[4] = {msg->data[i+4], msg->data[i+5], msg->data[i+6], msg->data[i+7]};
            float y = *(float *)arr2;
            unsigned char arr3[4] = {msg->data[i+8], msg->data[i+9], msg->data[i+10], msg->data[i+11]};
            float z = *(float *)arr3;
            pcl_pcd->points[cnt].x = x; pcl_pcd->points[cnt].y = y; pcl_pcd->points[cnt].z = z;
            MyPointXYZ tmp;
            tmp.x = x; tmp.y = y; tmp.z = z;
            point_buffer.push_back(tmp);
            cnt++;
        }
    }
    cnt = 0;

    //voxel filter from ROS msg
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2 msg_filtered;
    pcl_conversions::toPCL(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int select_w_s = (int)(msg->width * roi_w_rate_/100.0); int select_w_end = (int)(msg->width * (1-roi_w_rate_/100.0));
    int select_h_s = (int)(msg->height * roi_h_rate_/100.0); int select_h_end = (int)(msg->height * (1-roi_h_rate_/100.0));
    roi_cloud->points.resize((select_h_end-select_h_s)*(select_w_end-select_w_s));
    static int ii = 0;
    for(int m = select_h_s; m < select_h_end; m++)
    {
        for(int n = select_w_s; n < select_w_end; n++)
        {
            roi_cloud->points[ii].x = pcl_pcd->points[m*msg->width+n].x;
            roi_cloud->points[ii].y = pcl_pcd->points[m*msg->width+n].y;
            roi_cloud->points[ii].z = pcl_pcd->points[m*msg->width+n].z;
            ii++;
        }
    }
    ii = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setInputCloud(roi_cloud);
    voxelgrid.setLeafSize(0.05f,0.05f,0.05f);
    voxelgrid.filter(*cloud_after_voxelgrid);

    sensor_msgs::PointCloud2 points_output;
    pcl::toPCLPointCloud2(*cloud_after_voxelgrid, msg_filtered);
    pcl_conversions::fromPCL(msg_filtered, points_output);
    std::cout << "Origin PointCloud after voxel filtering size: " << points_output.data.size () << std::endl;
    points_output.header.frame_id = msg->header.frame_id;
    points_output.header.stamp = ros::Time::now();
    m_pubFilterPot.publish(points_output);

    if(0 == used_method_)
        return;

    //point after filter convert to pcl::PointXYZ format
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(msg_filtered, *pcl_filter);

    // select range of an interest area to fit a plane
    float x, y, z;
    std::vector<MyPointXYZ> xyz_vector;
    xyz_vector.clear();
    int start_width = (int)msg->width/4.0; int end_width = (int)msg->width/4.0*3.0;
    int start_height = (int)msg->height/10.0*4.0; int end_height = (int)msg->height;

    for(int i = start_height; i < end_height; i+= 5)
    {
        for(int j = start_width; j < end_width; j++)
        {
            x = point_buffer[i*msg->width+j].x;
            y = point_buffer[i*msg->width+j].y;
            z = point_buffer[i*msg->width+j].z;
            if(std::isnan(x) || std::isnan(y) || std::isnan(z))    //nan result
                continue;
            if(::fabs(x) < 0.00001 || ::fabs(y) < 0.00001 || ::fabs(z) < 0.00001)
                continue;
            MyPointXYZ point_xyz;
            point_xyz.x = x; point_xyz.y = y; point_xyz.z = z;
            xyz_vector.push_back(point_xyz);
        }
    }

    // show selected points
    sensor_msgs::PointCloud pcloud;
    pcloud.header.stamp = ros::Time::now();
    pcloud.header.frame_id = frame_id;
    pcloud.points.resize(xyz_vector.size());
    //pcloud.channels.resize(1);
    //pcloud.channels[0].name = "rgb";
    //pcloud.channels[0].values.resize(xyz_vector.size());
    for(unsigned int i = 0; i < xyz_vector.size(); i++)
    {
        pcloud.points[i].x = xyz_vector.at(i).x;
        pcloud.points[i].y = xyz_vector[i].y;
        pcloud.points[i].z = xyz_vector[i].z;
        //pcloud.channels[0].values[i] = i*10;
    }
    m_pubPoints.publish(pcloud);

      // aX+bY+cZ+d = 0, using RANSAC method, but seems not to work well
//    float ransac_plane[4];
//    CalTool::cal_ransac(xyz_vector, ransac_plane);

    double fitparam[4] = {0,0,0,0}; float aa, bb, cc;
    if(2 == used_method_)    // PCL ransac
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        plane_cloud->points.resize(xyz_vector.size());
        for(int cnt = 0; cnt < xyz_vector.size(); cnt++)
        {
            plane_cloud->points[cnt].x = xyz_vector.at(cnt).x;
            plane_cloud->points[cnt].y = xyz_vector.at(cnt).y;
            plane_cloud->points[cnt].z = xyz_vector.at(cnt).z;
        }
        std::cout << "Get ROI points size: " << xyz_vector.size() << std::endl;

        if(false == this->pcl_ransac_method(plane_cloud, fitparam))
            return;
    }
    else if(1 == used_method_)   //least square method
    {
        CalTool::least_square3D(xyz_vector, aa, bb, cc);
        std::cout << "Least square result: a=" << aa << " ,b=" << bb << " ,c=" << -1 << ", d=" << cc << std::endl;

        if(std::isnan(aa) || std::isnan(bb) || std::isnan(cc))
            return;
    }

    // ax+by+cz+d=   to-->  ax+by-z+c=0
    if(::fabs(fitparam[3]) > 0.00001)
    {
        aa = -fitparam[0]/fitparam[2];
        bb = -fitparam[1]/fitparam[2];
        cc = -fitparam[3]/fitparam[2];
        std::cout << "PCL ransac result: a=" << aa << " ,b=" << bb << " ,c=" << -1 << ", d=" << cc << std::endl;
    }

    //calculate the degree between normal vector and  Z axis
    MyPointXYZ plane_normal; plane_normal.x = aa; plane_normal.y = bb; plane_normal.z = -1;
    MyPointXYZ z_axis; z_axis.x = 0; z_axis.y = 0; z_axis.z = 1;
    float dot_result = CalTool::vector_dot(plane_normal, z_axis);
    if(dot_result < 0)
        plane_normal.x = -aa; plane_normal.y = -bb; plane_normal.z = 1;

    /*********** calculate the axis in the plane *************/
    // shadow point from O
    MyPointXYZ point_0; point_0.x = 0; point_0.y = 0; point_0.z = 0;
    MyPointXYZ shadow_point = CalTool::shadow_point(aa,bb,-1,cc, point_0);
    if(::fabs(aa*shadow_point.x+bb*shadow_point.y-shadow_point.z+cc) < 0.00001)
        std::cout << "shadow point: x = " <<shadow_point.x << "  y = " << shadow_point.y << "  z = " << shadow_point.z << std::endl;

    MyPointXYZ fitZ_axis, fitY_axis, fitX_axis;
    static MyPointXYZ fitZ_bk, fitY_bk, fitX_bk;
    if((shadow_point.x-point_0.x)/plane_normal.x < 0)
    {
        fitZ_axis.x = -(shadow_point.x-point_0.x);
        fitZ_axis.y = -(shadow_point.y-point_0.y);
        fitZ_axis.z = -(shadow_point.z-point_0.z);
    }
    else
    {
        fitZ_axis.x = shadow_point.x-point_0.x;
        fitZ_axis.y = shadow_point.y-point_0.y;
        fitZ_axis.z = shadow_point.z-point_0.z;
    }
    fitX_axis.x = aa-shadow_point.x;   //another point (-a/c, -b/c, z) in plane
    fitX_axis.y = bb-shadow_point.y;
    fitX_axis.z = CalTool::cal_z(aa,bb, aa,bb,cc)-shadow_point.z;
    fitY_axis = CalTool::vector_cross(fitX_axis, fitZ_axis);
    //normalize, and get the rotation matrix from eye(3)
    fitX_axis = CalTool::get_unit(fitX_axis);
    fitY_axis = CalTool::get_unit(fitY_axis);
    fitZ_axis = CalTool::get_unit(fitZ_axis);

    //draw plane axis
    geometry_msgs::PolygonStamped polytest;
    polytest.header.frame_id = frame_id;
    polytest.header.stamp = ros::Time::now();
    geometry_msgs::Point32 pointT[3];
    pointT[0].x = shadow_point.x; pointT[0].y = shadow_point.y; pointT[0].z = shadow_point.z;
    pointT[1].x = aa; pointT[1].y = bb; pointT[1].z = CalTool::cal_z(aa,bb, aa,bb,cc);
    pointT[2].x = point_0.x; pointT[2].y = point_0.y; pointT[2].z = point_0.z;
    polytest.polygon.points.push_back(pointT[0]);
    polytest.polygon.points.push_back(pointT[1]);
    polytest.polygon.points.push_back(pointT[2]);
    m_pubTest.publish(polytest);

    // calculate camera height
    double zz = CalTool::dis_pt2plane(point_0, aa, bb, -1, cc);
    std::cout << "camera height: z = " << zz << std::endl;

    //create a new plane's tf under "camera_depth_optical_frame"
    static tf::TransformBroadcaster br_plane;
    tf::Transform tf_br;
    tf_br.setOrigin(tf::Vector3(shadow_point.x, shadow_point.y, shadow_point.z));
    tf::Quaternion q_br;
    float rpy_br[3];
    CalTool::cal_eulerRPY(fitX_axis, fitY_axis, fitZ_axis, rpy_br);
    q_br.setRPY(rpy_br[0], rpy_br[1], rpy_br[2]);
    tf_br.setRotation(q_br);
    br_plane.sendTransform(tf::StampedTransform(tf_br, ros::Time::now(), frame_id, "depth_plane_tf"));

    // tf transform between "plane" and "camera_depth_optical_frame"
    ros::Time last_error = ros::Time::now();
    std::string tf_error;
    static tf::TransformListener tf_;
    // make sure the transform frame is available
    while (ros::ok()
           && !tf_.waitForTransform("depth_plane_tf", frame_cam, ros::Time(), ros::Duration(0.1), ros::Duration(0.01),
                                    &tf_error))
    {
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now())
        {
            ROS_ERROR("Timed out waiting for transform from %s to %s , tf error: %s",
                      frame_cam.c_str(), "depth_plane_tf", tf_error.c_str());
            return;
        }
    }
    tf::StampedTransform transform;
    try
    {
        tf_.lookupTransform("depth_plane_tf",frame_cam, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }
    double tf_x = transform.getOrigin().x();
    double tf_y = transform.getOrigin().y();
    double tf_z = transform.getOrigin().z();
    static double tf_z_bk = camera_z_;
    //std::cout << "tf x : " << tf_x << "  tf y: " << tf_y << "  tf_z:" << tf_z << std::endl;

    // get the basis matrix for the rotation
    tf::Matrix3x3 basis_ = transform.getBasis();
    tf::Quaternion q;
    basis_.getRotation(q);
    double r_, p_, y_;
    basis_.getRPY(r_, p_, y_);

    if(fabs(tf_z-camera_z_) > cali_z_diff_)
    {
        fitZ_axis = fitZ_bk; fitY_axis = fitY_bk; fitX_axis = fitX_bk;
        tf_z = tf_z_bk;
    }
    else if(fabs(camera_z_) < 0.00001)   // initial value = 0
    {
        fitZ_bk = fitZ_axis; fitY_bk = fitY_axis; fitX_bk = fitX_axis;
        tf_z_bk = tf_z;
    }
    else
    {
        fitZ_bk = fitZ_axis; fitY_bk = fitY_axis; fitX_bk = fitX_axis;
        tf_z_bk = tf_z;
    }

    static double r_bk_ = camera_roll_;
    static double p_bk_ = camera_pitch_;
    if(fabs(r_ - camera_roll_)  > cali_angle_diff_ || fabs(p_ - camera_pitch_) > cali_angle_diff_ || fabs(y_) > 0.0001)
    {
        r_ = r_bk_; p_ = p_bk_; y_ = 0;
    }
    else
    {
        r_bk_ = r_; p_bk_ = p_;
    }

    rgbd_calibration::RgbdMsg result_msg;
    result_msg.z_height = CalTool::kalman_filter_z(tf_z);
    result_msg.rgbd_roll = CalTool::kalman_filter_r(r_);
    result_msg.rgbd_pitch = CalTool::kalman_filter_p(p_);
    result_msg.rgbd_yaw = y_;
    m_pubCalibrationRes.publish(result_msg);

    // broadcast "camera_link" to "base_footprint"
    static tf::TransformBroadcaster br_cam;
    tf::Transform tf_cam;
    tf_cam.setOrigin(tf::Vector3(origin_x_, 0, tf_z));
    tf::Quaternion q_cam;
    q_cam.setRPY(r_, p_, 0);
    tf_cam.setRotation(q_cam);
    br_cam.sendTransform(tf::StampedTransform(tf_cam, ros::Time::now(), "base_footprint", frame_cam));

    // publish camera pose
    Eigen::Quaterniond quat(q.getW(), q.getX(), q.getY(), q.getZ());
    Eigen::Matrix3d rot_plane_cam = quat.toRotationMatrix();
    std::cout << "rotation between palne and camera:\n" << rot_plane_cam << std::endl;
    geometry_msgs::PoseStamped cam_pose;
    cam_pose.pose.position.y = tf_x;
    cam_pose.pose.position.y = tf_y;
    cam_pose.pose.position.z = tf_z;
    cam_pose.pose.orientation.w = q.getW();
    cam_pose.pose.orientation.x = q.getX();
    cam_pose.pose.orientation.y = q.getY();
    cam_pose.pose.orientation.z = q.getZ();
    cam_pose.header.stamp = ros::Time::now();
    cam_pose.header.frame_id = "depth_plane_tf";
    m_pubPose.publish(cam_pose);

    double end_time = double(clock());
    std::cout << "The calculate time = " << (end_time - start_time)/1000 << " ms" << std::endl;

//    pcl::PCDWriter writer;
//    writer.write ("pcl_point_flat.pcd", *pcl_filter, false);
    std::cout << "\n";
}

bool RgbdCalib::pcl_ransac_method(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double fitparam[4])
{
    // RanSac
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(sac_threshold_);
    seg.setMaxIterations(sac_iteration_);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(0 == inliers->indices.size())
    {
        PCL_ERROR("Could not estimate a planar model.");
        return false;
    }

    fitparam[0] = coefficients->values[0];
    fitparam[1] = coefficients->values[1];
    fitparam[2] = coefficients->values[2];
    fitparam[3] = coefficients->values[3];

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rgbd_calibration");

    RgbdCalib rgbd_calib_;

    ros::spin();

    return 0;
} 

