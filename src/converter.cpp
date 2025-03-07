#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("pointcloud_converter") {
        // 参数配置
        input_topic_ = this->declare_parameter<std::string>("input_topic", "/input_cloud");
        output_topic_ = this->declare_parameter<std::string>("output_topic", "/output_cloud2");

        // 创建订阅者和发布者
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            input_topic_, 10,
            std::bind(&PointCloudConverter::callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), 
            "Node initialized. Converting '%s' -> '%s'", 
            input_topic_.c_str(), 
            output_topic_.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud::SharedPtr msg) {
        try {
            // 转换点云格式
            sensor_msgs::msg::PointCloud2 output;
            
            // 使用ROS2官方转换函数
            if(sensor_msgs::convertPointCloudToPointCloud2(*msg, output)) {
                // 保持时间戳一致
                output.header.stamp = msg->header.stamp;
                output.header.frame_id = msg->header.frame_id;
                
                // 发布转换后的消息
                publisher_->publish(output);
                
                // 调试输出
                RCLCPP_DEBUG(this->get_logger(), 
                    "Converted cloud with %lu points", 
                    msg->points.size());
            } else {
                RCLCPP_ERROR(this->get_logger(), "PointCloud conversion failed!");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Conversion error: %s", e.what());
        }
    }

    // 成员变量
    std::string input_topic_;
    std::string output_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}