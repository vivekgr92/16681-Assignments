#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //cv::Point p;
  cv::Point p[];
  sensor_msgs::ImagePtr toImageMsg() const;
  void toImageMsg(sensor_msgs::Image& ros_image) const;


public:
  
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void setTagLocations(float x_det, float y_det, float z_det)
  {
	  //TODO: Update tag locations
    float result_x;
    float result_y;

    // Mapping the x and y value from -1 to 1 to 0 to 640 and 0 to 480
    result_x=(x_det+1)*(640-0)/(1+1)+0;
    result_y=(y_det+1)*(480-0)/(1+1)+0;

    // Pushing the values to the vector
    x_arr.push_back (result_x);
    y_arr.push_back (result_y);
    ROS_INFO("PUSH BACK");
    

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    
    float x,y;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	 //TODO: Draw circles at tag locations on image. 

     ROS_INFO("Size [%x]",x_arr.size()); 
     ROS_INFO("Size [%x]",y_arr.size());

    // Drawing Multiple Circles
    

     for (int i=0;i<x_arr.size(),i<y_arr.size(); ++i)
    {
     cv::circle(cv_ptr->image, cv::Point(x_arr[i],y_arr[i]),15, cv::Scalar( 0, 255, 0),3); 
    }


    // while (!x_arr.empty()&&!y_arr.empty())
    // {

    //   x=x_arr.back();
    //   x_arr.pop_back();
    //   y=y_arr.back();
    //   y_arr.pop_back();
    //   cv::circle(cv_ptr->image, cv::Point(x,y),30, cv::Scalar( 0, 255, 0),3);
    // }
    
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // TODO:Output modified video stream

    // Convert the modified frames into sensor_msgs::Image message and publish it using image_pub
    
    image_pub_.publish(cv_ptr->toImageMsg());


  }

private:
  float x_loc ,y_loc;
  std::vector<float> x_arr;
  std::vector<float> y_arr;
};
