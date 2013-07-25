#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

#include "ardrone_autonomy/Navdata.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;
static const char WINDOW[] = "Image window";

class ImageConverter {
  ros::NodeHandle nh_;
  ros::NodeHandle navdata_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber navdata_sub_;
  unsigned int drone_state_;
  unsigned int tags_count_;
  vector<unsigned int> tags_xc_;
  vector<unsigned int> tags_yc_;

public:
  /* constructor method called on instantiation of ImageConverter */
  ImageConverter(string topic) : it_(nh_) {
    /* subscribe to ardrone camera feed and publish it out as a new message */
    image_pub_ = it_.advertise("converted", 1);
    image_sub_ = it_.subscribe(topic, 1, &ImageConverter::imageCb, this);
    navdata_sub_ = navdata_.subscribe("/ardrone/navdata", 10, &ImageConverter::navdataCallback, this);

    cv::namedWindow(WINDOW);
  }

  // destructor method called on destroy of ImageConverter instances
  ~ImageConverter() {
    cv::destroyWindow(WINDOW);
  }

  /* takes an image_transport image and converts it to cvMat */
  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Point center;
    cv::Scalar color = cv::Scalar(14, 200, 60);
    center.x = tags_xc_[0];
    center.y = tags_yc_[0];
    
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::circle(cv_ptr -> image, center, 5, color, 2);
    // cv::imshow(WINDOW, cv_ptr -> image);
    // cv::waitKey(3);
    image_pub_.publish(cv_ptr -> toImageMsg());
  }
  
  /* takes a navdata message and extracts useful things*/
  void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr &msg) {
    drone_state_ = msg -> state;
    tags_count_ = msg -> tags_count;

    tags_xc_ = msg -> tags_xc;
    tags_yc_ = msg -> tags_yc;
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "image_converter");
  ImageConverter converter("/ardrone/front/image_raw");
  cout << "This program is converting messages" << endl;
  ros::spin();
  return 0;
}
