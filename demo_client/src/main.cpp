#include <iostream>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "detect/obj_detect.h"

int main(int argc,char *argv[]) {
    ros::init(argc,argv,"demo_client");
    ros::NodeHandle handle;

    ros::ServiceClient client=handle.serviceClient<detect::obj_detect>("detect_srv");

    detect::obj_detect srv;

    cv::Mat img=cv::imread(argv[1]);

    sensor_msgs::Image input_img;
    cv_bridge::CvImage imgBridge;
    std_msgs::Header header;
    header.seq=0;
    header.stamp=ros::Time::now();
    imgBridge=cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,img);
    imgBridge.toImageMsg(input_img);

    srv.request.img=input_img;
    for (unsigned i=0;i<100000;i++) {
        if (client.call(srv)) {
            ROS_INFO("label %d (%d %d) (%d %d)", srv.response.lablel,
            srv.response.top_left_x,srv.response.top_left_y,
            srv.response.bottom_right_x,srv.response.bottom_right_y);
        } else {
            ROS_INFO("FAILED");
        }
        std::cout<<ros::Time::now()<<std::endl;
    }

    return 0;
}
