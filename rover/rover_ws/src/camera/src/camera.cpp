#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class GstCameraNode : public rclcpp::Node
{
public:
    GstCameraNode() : Node("camera")
    {
        using std::placeholders::_1;

        std::string video_device = this->declare_parameter<std::string>("video_device", "/dev/video0");
        std::string camera_name = this->declare_parameter<std::string>("camera_name", "camera");
        std::string output_topic = camera_name + "/color/image_raw/compressed";

        // 通信安定性重視のQoS設定
        // - BEST_EFFORT: 再送しない（フレーム落としてOK）
        // - VOLATILE: 過去データを保持しない
        // - Keep Last(1): 常に最新の1フレームのみ
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            output_topic, qos);

        // Webカメラから送られるMJPGをデコードせず直接配信（CPU使用率最小化）
        // leaky=downstream で古いフレームを破棄、常に最新を転送
        std::string pipeline_desc =
            "v4l2src device=" + video_device + " io-mode=4 ! "
            "image/jpeg,width=1280,height=720,framerate=30/1 ! "
            "queue max-size-buffers=2 leaky=downstream silent=true ! "
            "appsink name=appsink_compressed emit-signals=true sync=false max-buffers=1 drop=true";

        GError *err = nullptr;
        pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &err);
        if (err) {
            RCLCPP_ERROR(this->get_logger(), "Pipeline error: %s", err->message);
            g_error_free(err);
            return;
        }

        appsink_compressed_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink_compressed"));

        gst_app_sink_set_emit_signals(appsink_compressed_, true);
        gst_app_sink_set_max_buffers(appsink_compressed_, 1);
        gst_base_sink_set_sync(GST_BASE_SINK(appsink_compressed_), false);

        g_signal_connect(appsink_compressed_, "new-sample",
                         G_CALLBACK(&GstCameraNode::on_new_sample_compressed_static), this);

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started.");
    }

    ~GstCameraNode()
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(appsink_compressed_);
        gst_object_unref(pipeline_);
    }

private:
    static GstFlowReturn on_new_sample_compressed_static(GstAppSink *sink, gpointer user_data)
    {
        return static_cast<GstCameraNode*>(user_data)->on_new_sample_compressed(sink);
    }

    GstFlowReturn on_new_sample_compressed(GstAppSink *sink)
    {
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample)
            return GST_FLOW_ERROR;

        GstBuffer *buffer = gst_sample_get_buffer(sample);

        // JPEGデータをCompressedImageとして直接パブリッシュ
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_frame";
        msg.format = "jpeg";

        // JPEGバッファの中身をコピー
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        msg.data.resize(map.size);
        memcpy(msg.data.data(), map.data, map.size);
        gst_buffer_unmap(buffer, &map);

        pub_compressed_->publish(msg);

        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    GstElement *pipeline_;
    GstAppSink *appsink_compressed_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
};

int main(int argc, char **argv)
{
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GstCameraNode>());
    rclcpp::shutdown();
    return 0;
}
