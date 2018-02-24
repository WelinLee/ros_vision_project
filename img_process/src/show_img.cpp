#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        ROS_INFO("Image info: height-%d, width-%d, encoding-%s", msg->height, msg->width, (msg->encoding).c_str());
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_show");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image_process", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("view");
}
