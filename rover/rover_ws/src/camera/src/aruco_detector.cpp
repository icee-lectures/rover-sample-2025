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
    ArucoDetectorNode() : Node("aruco_detector"), last_process_time_(this->now())
    {
        using std::placeholders::_1;

        dictionary_id_ = this->declare_parameter<int>("dictionary_id", cv::aruco::DICT_4X4_50);
        publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);
        int process_interval_ms = this->declare_parameter<int>("process_interval_ms", 500);
        detection_scale_ = this->declare_parameter<double>("detection_scale", 0.5);
        use_grayscale_ = this->declare_parameter<bool>("use_grayscale", true);
        jpeg_quality_ = this->declare_parameter<int>("debug_jpeg_quality", 50);
        
        process_interval_ = std::chrono::milliseconds(process_interval_ms);
        std::string image_topic = this->declare_parameter<std::string>("image_topic", "camera/image_raw/compressed");
        std::string namespace_prefix = this->declare_parameter<std::string>("namespace_prefix", "aruco");
        std::string aruco_topic = namespace_prefix + "/markers";
        std::string debug_image_topic = namespace_prefix + "/debug/image_raw/compressed";

        dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id_);
        detector_params_ = cv::aruco::DetectorParameters::create();
        
        // 検出パラメータの最適化
        detector_params_->adaptiveThreshWinSizeMin = 5;
        detector_params_->adaptiveThreshWinSizeMax = 15;
        detector_params_->adaptiveThreshWinSizeStep = 4;

        // QoS設定: KeepLast(1)でバッファリングを最小化
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        sub_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic, qos,
            std::bind(&ArucoDetectorNode::onImage, this, _1));

        aruco_pub_ = this->create_publisher<aruco_msgs::msg::MarkerArray>(
            aruco_topic, rclcpp::SystemDefaultsQoS());

        if (publish_debug_image_) {
            debug_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
                debug_image_topic, qos);
        }

        RCLCPP_INFO(this->get_logger(), 
            "ArUco detector started: interval=%dms, scale=%.2f, grayscale=%s, debug=%s",
            process_interval_ms, detection_scale_, 
            use_grayscale_ ? "yes" : "no",
            publish_debug_image_ ? "yes" : "no");
    }

private:
    void onImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        // Throttle processing to reduce system load
        auto current_time = this->now();
        auto elapsed = current_time - last_process_time_;
        if (elapsed.to_chrono<std::chrono::milliseconds>() < process_interval_) {
            return;
        }
        last_process_time_ = current_time;

        if (msg->format.find("jpeg") == std::string::npos && msg->format.find("jpg") == std::string::npos) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Incoming image is not JPEG encoded; skipping");
            return;
        }

        // グレースケールでデコード（カラーが不要な場合）
        int decode_flags = use_grayscale_ ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR;
        cv::Mat frame = cv::imdecode(msg->data, decode_flags);
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to decode compressed image");
            return;
        }

        // 画像のスケールダウン（ArUco検出は低解像度でも動作）
        cv::Mat detection_frame;
        if (detection_scale_ < 1.0) {
            cv::resize(frame, detection_frame, cv::Size(), detection_scale_, detection_scale_, cv::INTER_AREA);
        } else {
            detection_frame = frame;
        }

        // グレースケール変換（カラーでデコードした場合）
        cv::Mat gray_frame;
        if (!use_grayscale_ && detection_frame.channels() == 3) {
            cv::cvtColor(detection_frame, gray_frame, cv::COLOR_BGR2GRAY);
        } else {
            gray_frame = detection_frame;
        }

        // ArUco検出
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray_frame, dictionary_, corners, ids, detector_params_);

        // コーナー座標を元のスケールに戻す
        if (detection_scale_ < 1.0 && !corners.empty()) {
            float scale_factor = 1.0f / detection_scale_;
            for (auto& marker_corners : corners) {
                for (auto& corner : marker_corners) {
                    corner.x *= scale_factor;
                    corner.y *= scale_factor;
                }
            }
        }

        // Publish detected markers
        auto marker_array_msg = aruco_msgs::msg::MarkerArray();
        marker_array_msg.header = msg->header;

        if (!ids.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Detected %zu markers. First ID: %d", ids.size(), ids.front());

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

        // デバッグ画像のパブリッシュ（オプション）
        if (publish_debug_image_ && debug_pub_) {
            cv::Mat annotated = frame.clone();
            
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(annotated, corners, ids);
            }

            std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
            std::vector<uchar> encoded;
            if (!cv::imencode(".jpg", annotated, encoded, encode_params)) {
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
    bool use_grayscale_;
    double detection_scale_;
    int jpeg_quality_;
    std::chrono::milliseconds process_interval_;
    rclcpp::Time last_process_time_;
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
