#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>

class CameraFallbackNode : public rclcpp::Node {
public:
    CameraFallbackNode() : Node("camera_fallback") {
        // パラメータの宣言
        this->declare_parameter<std::string>("input_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("output_topic", "/camera_fallback/color/image_raw/compressed");
        this->declare_parameter<int>("output_width", 320);
        this->declare_parameter<int>("output_height", 240);
        this->declare_parameter<int>("output_ms", 500); // ミリ秒間隔で送信
        this->declare_parameter<int>("jpeg_quality", 30);  // 0-100, 高いほど品質が良い

        // パラメータを取得
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        output_width_ = this->get_parameter("output_width").as_int();
        output_height_ = this->get_parameter("output_height").as_int();
        output_ms_ = this->get_parameter("output_ms").as_int();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

        // QoS: サブスクライブ用とパブリッシュ用を分ける
        rclcpp::QoS input_qos(rclcpp::KeepLast(1));
        input_qos.best_effort();
        input_qos.durability_volatile();

        // サブスクライバーを作成（deadline は設定しない）
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic_,
            input_qos,
            std::bind(&CameraFallbackNode::image_callback, this, std::placeholders::_1)
        );

        // 出力用 QoS: 出力頻度の監視のために deadline を設定
        rclcpp::QoS output_qos(rclcpp::KeepLast(1));
        output_qos.best_effort();
        output_qos.durability_volatile();
        output_qos.deadline(std::chrono::milliseconds(output_ms_));

        // パブリッシャーを作成（圧縮イメージで帯域削減）
        image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            output_topic_, 
            output_qos
        );

        // 出力をミリ秒単位で制御。初回は即時送信できるように過去時刻に設定
        last_publish_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(output_ms_);

        RCLCPP_INFO(this->get_logger(), 
            "Camera Fallback Node started");
        RCLCPP_INFO(this->get_logger(), 
            "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), 
            "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), 
            "Output size: %dx%d, interval %d ms", output_width_, output_height_, output_ms_);
        RCLCPP_INFO(this->get_logger(), 
            "JPEG quality: %d", jpeg_quality_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
    
    std::string input_topic_;
    std::string output_topic_;
    int output_width_;
    int output_height_;
    int output_ms_;
    int jpeg_quality_;
    std::chrono::steady_clock::time_point last_publish_time_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr input_msg) {
        // nミリ秒以内に次のフレームが来た場合は破棄する
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time_).count();
        if (elapsed < output_ms_) {
            return;
        }
        last_publish_time_ = now;

        try {
            // ROS 2メッセージをOpenCV Matに変換
            cv::Mat original = cv_bridge::toCvShare(input_msg, "bgr8")->image;

            // リサイズ
            cv::Mat resized;
            cv::resize(original, resized, cv::Size(output_width_, output_height_), 
                      0, 0, cv::INTER_LINEAR);

            // JPEG圧縮のパラメータを設定
            std::vector<int> params;
            params.push_back(cv::IMWRITE_JPEG_QUALITY);
            params.push_back(jpeg_quality_);

            // JPEG エンコードして CompressedImage で送信（帯域削減）
            std::vector<uchar> buf;
            cv::imencode(".jpg", resized, buf, params);

            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header = input_msg->header;
            comp_msg.format = "jpeg";
            comp_msg.data.assign(buf.begin(), buf.end());

            image_pub_->publish(comp_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraFallbackNode>());
    rclcpp::shutdown();
    return 0;
}
