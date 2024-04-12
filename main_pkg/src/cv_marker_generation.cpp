#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <custom_interfaces/srv/set_obj_id.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

class ArucoMarkerGenerator : public rclcpp::Node
{
public:
  ArucoMarkerGenerator() : Node("aruco_marker_generator")
  {
    service_ = create_service<custom_interfaces::srv::SetObjId>(
      "generate_marker",
      std::bind(&ArucoMarkerGenerator::generateMarkerCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

private:
  void generateMarkerCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<custom_interfaces::srv::SetObjId::Request> request,
    std::shared_ptr<custom_interfaces::srv::SetObjId::Response> response)
  {
    (void)request_header;

    // Generate the ArUco marker
    int markerSize = 6; // size of the marker in pixels
    int markerId = request->objid;   // identifier of the marker
    int borderSize = 1; // size of the white border around the marker
    int imageSize = 200; // size of the output image in pixels

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    Mat markerImage;
    aruco::generateImageMarker(dictionary, markerId, imageSize, markerImage, borderSize);
    //aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
    // Save the image to a file
    std::string filename = "marker.png";
    imwrite(filename, markerImage);

    // Convert the OpenCV image to a ROS image message
    cv_bridge::CvImage cvImage;
    cvImage.header.stamp = this->get_clock()->now();
    cvImage.header.frame_id = "aruco_marker";
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    cvImage.image = markerImage;

    // Publish the generated marker image as the response
    response->image = *(cvImage.toImageMsg());
  }

  rclcpp::Service<sensor_msgs::srv::Image>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMarkerGenerator>());
  rclcpp::shutdown();
  return 0;
}
