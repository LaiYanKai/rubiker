#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "rpi_cam_pub");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera", 1);
  cv::VideoCapture cap(0); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
  {  
    ROS_ERROR("Rpi Cam fail");
    return -1;
  }
cap.set(cv::CAP_PROP_FRAME_WIDTH,320);
cap.set(cv::CAP_PROP_FRAME_HEIGHT,240);
  cv::Mat frame;
  // cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  // cv::waitKey(30);
  
  ros::Rate loop_rate(30);
  while (nh.ok()) {
    cap >> frame; // get a new frame from camera
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
