#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
static const char WINDOW[] = "Image window";

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_(nh_) {
    /* subscribe to ardrone camera feed and publish it out as a new message */
    image_pub_ = it_.advertise("converted", 1);
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  // destructor method called on destroy of ImageConverter instances
  ~ImageConverter() {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::imshow(WINDOW, cv_ptr -> image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr -> toImageMsg());
  }
  
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic ;
  cout << "This program is converting messages" << endl;
  ros::spin();
  return 0;
}
