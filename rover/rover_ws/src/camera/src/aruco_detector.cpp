#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <aruco_msgs/msg/marker_array.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

class ArucoDetectorNode : public rclcpp::Node
{
public:
    ArucoDetectorNode() : Node("aruco_detector")
    {
        using std::placeholders::_1;

        dictionary_id_ = this->declare_parameter<int>("dictionary_id", cv::aruco::DICT_4X4_50);
        publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);
        std::string image_topic = this->declare_parameter<std::string>("image_topic", "camera/image_raw/compressed");
        std::string namespace_prefix = this->declare_parameter<std::string>("namespace_prefix", "aruco");
        std::string aruco_topic = namespace_prefix + "/markers";
        std::string debug_image_topic = namespace_prefix + "/debug/image_raw/compressed";

        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id_);
        detector_params_ = cv::aruco::DetectorParameters::create();

        sub_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic, rclcpp::SensorDataQoS(),
            std::bind(&ArucoDetectorNode::onImage, this, _1));

        aruco_pub_ = this->create_publisher<aruco_msgs::msg::MarkerArray>(
            aruco_topic, rclcpp::SystemDefaultsQoS());

        if (publish_debug_image_) {
            debug_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                debug_image_topic, rclcpp::SystemDefaultsQoS());
        }

        RCLCPP_INFO(this->get_logger(), "Aruco detector node started. Subscribing to camera/image_raw/compressed");
    }

private:
    void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        if (msg->format.find("jpeg") == std::string::npos && msg->format.find("jpg") == std::string::npos) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Incoming image is not JPEG encoded; skipping");
            return;
        }

        // Decode JPEG data into an OpenCV matrix.
        cv::Mat frame = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to decode compressed image");
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);

        // Publish detected markers
        auto marker_array_msg = aruco_msgs::msg::MarkerArray();
        marker_array_msg.header = msg->header;

        if (ids.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No ArUco markers detected");
        } else {
            RCLCPP_INFO(this->get_logger(), "Detected %zu markers. First ID: %d", ids.size(), ids.front());

            for (size_t i = 0; i < ids.size(); ++i) {
                aruco_msgs::msg::Marker marker;
                marker.id = ids[i];
                marker.confidence = 1.0;

                // Set marker pose (default orientation)
                marker.pose.pose.position.x = 0.0;
                marker.pose.pose.position.y = 0.0;
                marker.pose.pose.position.z = 0.0;
                marker.pose.pose.orientation.x = 0.0;
                marker.pose.pose.orientation.y = 0.0;
                marker.pose.pose.orientation.z = 0.0;
                marker.pose.pose.orientation.w = 1.0;

                marker_array_msg.markers.push_back(marker);
            }
        }

        aruco_pub_->publish(marker_array_msg);

        if (publish_debug_image_ && debug_pub_) {
            cv::Mat annotated = frame.clone();
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(annotated, corners, ids);
            }

            std::vector<uchar> encoded;
            if (!cv::imencode(".jpg", annotated, encoded)) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to encode debug image");
                return;
            }

            auto out_msg = sensor_msgs::msg::CompressedImage();
            out_msg.header = msg->header;
            out_msg.format = "jpeg";
            out_msg.data = std::move(encoded);
            debug_pub_->publish(std::move(out_msg));
        }
    }

    int dictionary_id_;
    bool publish_debug_image_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_;
    rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr aruco_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
