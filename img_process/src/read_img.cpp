#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <sstream>
#include <iostream>
#include <std_msgs/Int8.h>

int main(int argc, char** argv)
{
    if(argv[1] == NULL)
    {
        ROS_INFO("No argument input!\n");
        return 0;
    }

    ros::init(argc, argv, "image_read");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img = it.advertise("image_process", 1);
    ros::Publisher pub_bound = nh.advertise<std_msgs::Int8>("boundwidth", 1000);

    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat gray;
    cv::Mat edges;
    cv::Mat show_img;
    sensor_msgs::ImagePtr msg;

    int boundwidth;
    int img_row = image.rows;
    int img_col = image.cols;
    ROS_INFO("Image size = %d , %d", img_row, img_col);

    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, edges, 76, 255, CV_THRESH_OTSU);    //76 could be dynamically changed

    ros::Rate loop_rate(10);
    for(int startline = 0; startline < img_row; startline++)
    {
        uchar *imgdata = edges.ptr<uchar>(startline);
        int colsNumber = edges.cols * edges.channels();
        int start_point, end_point;
        for(int cols = 0; cols < colsNumber; cols++)
        {
            if (0 == imgdata[cols])
            {
                start_point = cols;
                break;
            }
        }
        for(int cols = colsNumber-1; cols >= 0; cols--)
        {
            if (0 == imgdata[cols])
            {
                end_point = cols;
                break;
            }
        }
        cv::Point pt1, pt2;
        pt1.y = startline;
        pt1.x = start_point;
        pt2.y = startline;
        pt2.x = end_point;

        cv::cvtColor(edges, show_img, CV_GRAY2BGR);
        cv::line(show_img, pt1, pt2, cv::Scalar(0,255,0), 2);
        boundwidth = end_point - start_point;
        ROS_INFO("BoundWidth = %d" , boundwidth);

        if(nh.ok())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show_img).toImageMsg();
            pub_img.publish(msg);

            std_msgs::Int8 msg_wd;
            msg_wd.data = boundwidth;
            pub_bound.publish(msg_wd);

            ros::spinOnce();
            loop_rate.sleep();
        }
        else
        {
            ROS_ERROR("ROS node got error!");
            break;
        }
    }

    ROS_INFO("End of the program");
}
