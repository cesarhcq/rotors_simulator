#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace std;
using namespace cv;
// using namespace aruco;

const float arucoSquareDimension = 1.00f;
const Size chessboardDimensions = Size(6,9);

void createArucoMarkers()
{
    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

    for(int i = 0; i < 1000 ; i++)
    {
      aruco::drawMarker(markerDictionary, i, 400, outputMarker, 1);
      ostringstream convert;
      string imageName = "4X4_1000_Marker_";
      convert << imageName << i << ".jpeg";
      imwrite(convert.str(), outputMarker);
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	//criando matrizes de imagem
    Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat src_gray, edges, HSV;

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distanceCoefficients;
    vector<int> markerIds;
    vector<vector<Point2f > > markerCorners, rejectedCandidates;

    //detector arudo parametros
    aruco::DetectorParameters parameters;
    //chamando dicionario
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

    vector<Vec3d> rotationVectors, translationVectors;

    aruco::detectMarkers(src, markerDictionary, markerCorners, markerIds);
    //aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

    for (int i = 0; i < markerIds.size(); i++)
    {
      //if (markerIds[i] == 269)
      //{
        //cout << " marcador: " << markerIds[i] << endl;
        aruco::drawDetectedMarkers(src, markerCorners, markerIds);
        //aruco::drawAxis(src, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
        //cout << " translation [" << markerIds[i] << "] = " << translationVectors[i] << " ; rotation:" << rotationVectors[i] << endl;
      //}

    }

    

    //cvtColor(src, src_gray, CV_RGB2GRAY);
    cvtColor(src, HSV, COLOR_BGR2HSV);

    Canny(HSV, edges, 50, 200, 3);

    cv::imshow("Aruco", src);
    //cv::imshow("HSV", HSV);
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
  //cv::destroyWindow("HSV");
  cv::destroyWindow("Aruco");
  //createArucoMarkers();

}

