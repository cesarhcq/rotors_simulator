#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
// using namespace aruco;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat src_gray, edges, HSV;

    //cvtColor(src, src_gray, CV_RGB2GRAY);
    cvtColor(src, HSV, COLOR_BGR2HSV);

    Canny(HSV, edges, 50, 200, 3);

    cv::imshow("HSV", HSV);
    cv::imshow("ImgGray", edges);
    cv::waitKey(30);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("ImgGray");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("bebop2/camera_base/image_raw", 1, imageCallback);

  ros::spin();
  cv::destroyWindow("ImgGray");
  cv::destroyWindow("HSV");
}

