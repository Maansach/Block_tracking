#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

#include "geometry_msgs/Twist.h"

//for binarisation function:
#include "DetectionCnam_codels.hpp"

static const std::string OPENCV_WINDOW = "Image window";

int32_t _r;
int32_t _g;
int32_t _b;
int32_t _seuil;

ros::NodeHandle *nh;

geometry_msgs::Twist cmd;

void image_Callback(const sensor_msgs::Image::ConstPtr& msg)
{
  ros::Time begin = ros::Time::now();
  //usleep(10000);
//  ros::Time end = ros::Time::now();

  //necessary for transform ros image type into opencv image type
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ROS_INFO("I have received image! ;-)");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  IplImage _ipl_img=cv_ptr->image;
  IplImage *ptr_ipl_img= &_ipl_img;

  //For see OpenCV Image:
  //
  // Update GUI Window
  //  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //  cv::waitKey(3);

  //declare a CvPoint
  CvPoint coord;

  nh->getParam("r", _r);
  nh->getParam("g", _g);
  nh->getParam("b", _b);
  nh->getParam("threshold", _seuil);

  //call binarisation method!
  coord = binarisation(ptr_ipl_img, _b, _g, _r,_seuil);

  if (coord.x > -1)
  {

  //print with ROS_INFO coordinate points.x points.y
  ROS_INFO("coord X: %d",coord.x);
  ROS_INFO("coord Y: %d",coord.y);

  ROS_INFO("r: %d", _r);
  ROS_INFO("g: %d", _g);
  ROS_INFO("b: %d", _b);
  ROS_INFO("threshold: %d", _seuil);

  //Calculate command to send:
  float cibleY = ptr_ipl_img->height * 3 / 4;

  float cmd_x_pixel_value= 2.0 / ptr_ipl_img->width;
  float cmd_y_pixel_value= 2.0 / (ptr_ipl_img->height - cibleY);

  cmd.angular.z= - ((coord.x * cmd_x_pixel_value) -1.0);
  cmd.linear.x = - ((coord.y - cibleY) * cmd_y_pixel_value);

//  //cmd.angular.z=1.0;
//  if (coord.y > cibleY)
//    cmd.linear.x = -0.5;
//  else
//    cmd.linear.x = 0.5;

  //ROS_INFO("cibleY: %f", cibleY);
  ROS_INFO("cmd.linear.x: %f",cmd.linear.x);
  ROS_INFO("cmd.angular.z: %f",cmd.angular.z);
  }
  else
  {
    cmd.angular.z=0.0;
    cmd.linear.x=0.0;
  }

  // Print timing information
  ros::Time end = ros::Time::now();
  ROS_INFO("begin: %d",begin.toNSec());
  ROS_INFO("end: %d",end.toNSec());
  ROS_INFO("duration: %d", (end.toNSec() - begin.toNSec()));

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");

  nh = new ros::NodeHandle;

  ros::Subscriber sub = nh->subscribe("/TTRK/CameraMain/image", 10000, image_Callback);

  // For see OpenCV Image:
  //  cv::namedWindow(OPENCV_WINDOW);

  nh->setParam("r", 213);
  nh->setParam("g", 103);
  nh->setParam("b", 4);
  nh->setParam("threshold", 40);

 //Initliaise publisher
  ros::Publisher chatter_pub = nh->advertise<geometry_msgs::Twist>("/TTRK/Motion_Controller", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
  chatter_pub.publish(cmd);
  ROS_INFO("I have published the command! ;-)");
  ros::spinOnce(); // ensure subscribe thread is executed once
  loop_rate.sleep();
  }

  //cv::destroyWindow(OPENCV_WINDOW);
  return 0;
}

