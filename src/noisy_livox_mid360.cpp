#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>
#include <cmath>

class NoisyLivoxMid360Node : public rclcpp::Node {
public:
    NoisyLivoxMid360Node() : Node("noisy_livox_mid360_node") {
        this->declare_parameter("min_noise", 0.002);
        min_noise_ = static_cast<float>(this->get_parameter("min_noise").as_double());

        gen_       = std::mt19937(std::random_device{}());
        dist_norm_ = std::normal_distribution<float>(0.0f, 1.0f);

        auto qos = rclcpp::QoS(10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox_mid360/points_raw", qos,
            std::bind(&NoisyLivoxMid360Node::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox_mid360/points", qos);

        RCLCPP_INFO(this->get_logger(),
            "Livox Mid-360 noise node started — model: piecewise radial Gaussian (datasheet-informed)");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto out = *msg;

        sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");

        sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");

        for (; in_x != in_x.end();
               ++in_x, ++in_y, ++in_z,
               ++out_x, ++out_y, ++out_z)
        {
            const float x = *in_x, y = *in_y, z = *in_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            const float dist = std::sqrt(x * x + y * y + z * z);
            if (dist < 1e-6f) continue;

            // Piecewise relative noise for Mid-360:
            // dist <= 6m -> 0.4% (yields ~2cm @ 5m)
            // dist > 6m  -> 0.7% (yields ~3.5cm @ 5m, increasing accuracy gap)
            float rel_noise = (dist <= 6.0f) ? 0.004f : 0.007f;

            const float sigma = std::max(min_noise_, rel_noise * dist);
            const float ratio = 1.0f + (sigma * dist_norm_(gen_)) / dist;

            *out_x = x * ratio;
            *out_y = y * ratio;
            *out_z = z * ratio;
        }

        pub_->publish(out);
    }

    float min_noise_;
    std::mt19937 gen_;
    std::normal_distribution<float> dist_norm_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyLivoxMid360Node>());
    rclcpp::shutdown();
    return 0;
}
