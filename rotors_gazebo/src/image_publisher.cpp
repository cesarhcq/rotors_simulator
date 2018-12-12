#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <ctime>
#include <string>
#include <stdio.h>

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

const float arucoSquareDimension = 0.40f;
const Size chessboardDimensions = Size(6,9);

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//float to string helper function
string floatToString(float number)
{
  //this function has a number input and string output
  std::stringstream ss;
  ss << number;
  return ss.str();
}

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

bool loadCameraCalibration(Mat& cameraMatrix, Mat& distanceCoefficients)
{
  ifstream myfile;

  myfile.open("/home/alantavares/aruco_landing_ws/src/rotors_simulator/calibrationFile.txt");
  if (myfile.is_open())
  {
    uint16_t rows;
    uint16_t columns;

    myfile >> rows;
    myfile >> columns;

    cameraMatrix = Mat(Size(columns, rows), CV_64F);

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < columns; c++)
      {

        double read = 0.0f;
        myfile >> read;
        cameraMatrix.at<double>(r, c) = read;
        //cout << cameraMatrix.at<double>(r, c) << endl;
      }
    }

    //Distance Coefficients
    myfile >> rows;
    myfile >> columns;

    distanceCoefficients = Mat(Size(columns, rows), CV_64F);

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < columns; c++)
      {
        double read = 0.0f;
        myfile >> read;
        distanceCoefficients.at<double>(r, c) = read;
        //cout << distanceCoefficients.at<double>(r, c) << endl;
      }
    }
    myfile.close();
    return true;
  }

  return false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

  	//-- configurar height and width da captura da camera
    //TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	//TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

  	//-- criando matrizes de imagem
    Mat src = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat src_gray, edges, HSV;

    //-- criando matrizes de calibração
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distanceCoefficients;

    vector<int> markerIds;
    vector<vector<Point2f > > markerCorners, rejectedCandidates;

    //-- carrega os arquivos de calibração de câmera
    loadCameraCalibration(cameraMatrix, distanceCoefficients);

    //-- detector arudo parametros
    aruco::DetectorParameters parameters;
    //-- chamando dicionario
    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);
    //-- criando vertores de rotação e translação
    vector<Vec3d> rotationVectors, translationVectors;

    aruco::detectMarkers(src, markerDictionary, markerCorners, markerIds);
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);

    for (int i = 0; i < markerIds.size(); i++)
    {
      if (markerIds[i] == 1)
      {
        //cout << " marcador: " << markerIds[i] << endl;
        aruco::drawDetectedMarkers(src, markerCorners, markerIds);
        aruco::drawAxis(src, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
        //putText(src, "translation (" + floatToString(translationVectors[i]) + "m , rotation ", 
    	//		Point(10, 30), 1.2, 1.2, Scalar(0, 255, 0), 2);
        cout << " translation [" << markerIds[i] << "] = " << translationVectors[i] << " ; rotation:" << rotationVectors[i] << endl;
      }

    }

    putText(src, "X", 
    	Point(src.cols / 2, src.rows / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);

    

    //-- Imgem size is 856 x 480
    //cout << " img-size: " << src.size() << endl;

    //-- mostra imagem aruco
    cv::imshow("Img-Aruco", src);
    
    //-- converter para escala de cinza
    //cvtColor(src, src_gray, CV_RGB2GRAY);
    //cv::imshow("ImgGray", src_gray);

    //-- converter para escala HSV
    //cvtColor(src, HSV, COLOR_BGR2HSV);
    //cv::imshow("ImgHSV", HSV);

    //-- converter para escala Edges
    //Canny(src, edges, 50, 200, 3);
    //cv::imshow("ImgEdges", edges);

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
  //cv::namedWindow("ImgGray");
  cv::namedWindow("Img-Aruco", WINDOW_AUTOSIZE);
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("bebop2/camera_base/image_raw", 1, imageCallback);

  ros::spin();
  //cv::destroyWindow("ImgGray");
  //cv::destroyWindow("HSV");
  cv::destroyWindow("Img-Aruco");
  //createArucoMarkers();

}

