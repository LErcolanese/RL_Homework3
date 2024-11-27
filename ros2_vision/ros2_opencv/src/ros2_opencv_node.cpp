#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
 
class ImageProcessorNode : public rclcpp::Node {
public:
  ImageProcessorNode() : Node("opencv_image_processor") {
    // Subscriber per l'immagine simulata
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/videocamera", 10,
        std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));
 
    // Publisher per l'immagine processata
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
  }
 
private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
  {
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

	
    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    

    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    
    //Filter by color
    params.filterByColor=false;
    params.blobColor=0;

    // Filter by Area.
    params.filterByArea = false;
    params.minArea = 0.1;
    //params.maxArea=5000;
    
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.9;
    
    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;
    
    //Mat im=imread(cv_ptr->image,IMREAD_GRAYSCALE);

    // Set up detector with params
    //SimpleBlobDetector detector(params);
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    
    // You can use the detector this way
    // detector.detect( im, keypoints);

    std::vector<KeyPoint> keypoints;
    detector->detect(cv_ptr->image,keypoints);
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints( cv_ptr->image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(0,255,0));

    // Update GUI Window
    cv::imshow("Image window", im_with_keypoints);
    cv::waitKey(3);

    // Output modified video stream
    publisher_.publish(cv_ptr->toImageMsg());
  }
};
//   {
//     // Convertire l'immagine ROS in un'immagine OpenCV
//     cv::Mat input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
 
//     // // Convertire l'immagine in scala di grigi
//     // cv::Mat gray_image;
//     // cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
 
//     // // Applicare un filtro gaussiano per ridurre il rumore
//     // cv::Mat blurred_image;
//     // cv::GaussianBlur(gray_image, blurred_image, cv::Size(5, 5), 0);
 

//     // Setup SimpleBlobDetector parameters.
//     cv::SimpleBlobDetector::Params params;
    
//     // Change thresholds
//     params.minThreshold = 0;
//     params.maxThreshold = 255;
    
//     // Filter by Area.
//     params.filterByArea = false;
//     params.minArea = 0.1;
    
//     // Filter by Circularity
//     params.filterByCircularity = true;
//     params.minCircularity = 0.8;
    
//     // Filter by Convexity
//     params.filterByConvexity = true;
//     params.minConvexity = 0.87;
    
//     // Filter by Inertia
//     params.filterByInertia = false;
//     params.minInertiaRatio = 0.01;

//     params.filterByColor = false;
//     params.blobColor=0;
 
//     // Set up detector with params
//     auto detector = cv::SimpleBlobDetector::create(params);

//     // Rilevamento blob
//     std::vector<cv::KeyPoint> keypoints;
//     detector->detect(input_image, keypoints);

 
//     // Disegnare i blob rilevati sull'immagine originale
//     cv::Mat output_image;
//     cv::drawKeypoints(input_image, keypoints, output_image, cv::Scalar(0, 255, 0),
//                       cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


//     // Update GUI Window
//     cv::imshow("Image window", output_image);
//     cv::waitKey(3);

//     // Output modified video stream
//     auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output_image).toImageMsg();
 
//     // Ripubblicare l'immagine processata
//     publisher_->publish(*processed_msg);
//     RCLCPP_INFO(this->get_logger(), "Processed image published with %lu blobs detected.", keypoints.size());
//   }
 
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
// };
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageProcessorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}