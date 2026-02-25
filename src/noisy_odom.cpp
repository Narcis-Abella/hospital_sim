#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <random>
#include <cmath>
#include <array>

// Minimum velocity threshold below which noise is suppressed.
// Avoids injecting noise when the robot is stationary (encoder deadband).
static constexpr double kVelocityThreshold = 0.001;  // m/s or rad/s

class NoisyOdomNode : public rclcpp::Node {
public:
    NoisyOdomNode() : Node("noisy_odom_node") {

        // lin_noise_ratio: linear velocity noise as fraction of Vx (wheel slip)
        // ang_noise_ratio: angular velocity noise as fraction of Wz (differential error)
        // yaw_drift_rate:  yaw random walk per metre travelled (rad/m)
        this->declare_parameter("lin_noise_ratio", 0.02);
        this->declare_parameter("ang_noise_ratio", 0.08);
        this->declare_parameter("yaw_drift_rate",  0.005);

        lin_ratio_  = this->get_parameter("lin_noise_ratio").as_double();
        ang_ratio_  = this->get_parameter("ang_noise_ratio").as_double();
        drift_rate_ = this->get_parameter("yaw_drift_rate").as_double();

        // ── Pose covariance (6×6 row-major, diagonal) ──────────────────────
        // Values estimated from AgileX Tracer 2 wheel encoder characterisation.
        // Large values (1e6) on z/roll/pitch: planar robot, these DOF unmeasured.
        // High yaw covariance (0.08): EKF should weight IMU heading over odometry.
        pose_cov_.fill(0.0);
        pose_cov_[0]  = 0.005;  // x     (m²)
        pose_cov_[7]  = 0.005;  // y     (m²)
        pose_cov_[14] = 1e6;    // z     (unmeasured)
        pose_cov_[21] = 1e6;    // roll  (unmeasured)
        pose_cov_[28] = 1e6;    // pitch (unmeasured)
        pose_cov_[35] = 0.08;   // yaw   (rad²) — high: defer to IMU

        // ── Twist covariance (6×6 row-major, diagonal) ─────────────────────
        twist_cov_.fill(0.0);
        twist_cov_[0]  = 0.001;   // vx  (m²/s²) — reliable encoder measurement
        twist_cov_[7]  = 0.0001;  // vy  (m²/s²) — near-zero (non-holonomic)
        twist_cov_[14] = 1e6;     // vz  (unmeasured)
        twist_cov_[21] = 1e6;     // wx  (unmeasured)
        twist_cov_[28] = 1e6;     // wy  (unmeasured)
        twist_cov_[35] = 0.05;    // wz  (rad²/s²) — imprecise differential drive

        gen_       = std::mt19937(std::random_device{}());
        dist_norm_ = std::normal_distribution<double>(0.0, 1.0);

        // TF broadcaster: publishes odom_enc → base_footprint_enc.
        // This is a separate encoder-only frame consumed by kinematic_icp as a
        // motion prior, bypassing the EKF to avoid circular dependency:
        //   kinematic_icp prior ← odom_enc  (raw encoders)
        //   EKF output          → odom      (fused estimate)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Reliable QoS: matches parameter_bridge default output.
        auto qos = rclcpp::QoS(10);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", qos,
            std::bind(&NoisyOdomNode::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", qos);

        RCLCPP_INFO(this->get_logger(),
            "Wheel odometry noise node started — slip: %.1f%% lin / %.1f%% ang | "
            "yaw drift: %.4f rad/m | TF: odom_enc → base_footprint_enc",
            lin_ratio_ * 100.0, ang_ratio_ * 100.0, drift_rate_);
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto out = *msg;

        const double vx = out.twist.twist.linear.x;
        const double wz = out.twist.twist.angular.z;

        // 1. Linear slip noise: proportional to forward velocity.
        if (std::abs(vx) > kVelocityThreshold) {
            out.twist.twist.linear.x += std::abs(vx) * lin_ratio_ * dist_norm_(gen_);
        }

        // 2. Angular noise: proportional to yaw rate (differential drive error).
        if (std::abs(wz) > kVelocityThreshold) {
            out.twist.twist.angular.z += std::abs(wz) * ang_ratio_ * dist_norm_(gen_);
        }

        // 3. Yaw random walk: accumulated drift proportional to distance travelled.
        // yaw_drift_ is intentionally unbounded — models real encoder heading drift
        // that compounds over distance without loop-closure correction.
        const double pos_x = out.pose.pose.position.x;
        const double pos_y = out.pose.pose.position.y;

        if (!first_msg_) {
            const double dx        = pos_x - last_x_;
            const double dy        = pos_y - last_y_;
            const double dist_step = std::sqrt(dx * dx + dy * dy);

            if (dist_step > kVelocityThreshold) {
                yaw_drift_ += drift_rate_ * dist_step * dist_norm_(gen_);
            }
        }

        first_msg_ = false;
        last_x_    = pos_x;
        last_y_    = pos_y;

        // 4. Apply accumulated yaw drift to pose orientation.
        // Planar assumption (Tracer 2 on flat floor): quaternion roll=0, pitch=0.
        // Extract yaw from quaternion, add drift, reconstruct as pure-yaw quaternion.
        const double qx = out.pose.pose.orientation.x;
        const double qy = out.pose.pose.orientation.y;
        const double qz = out.pose.pose.orientation.z;
        const double qw = out.pose.pose.orientation.w;

        const double current_yaw = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));
        const double noisy_yaw = current_yaw + yaw_drift_;

        out.pose.pose.orientation.x = 0.0;
        out.pose.pose.orientation.y = 0.0;
        out.pose.pose.orientation.z = std::sin(noisy_yaw / 2.0);
        out.pose.pose.orientation.w = std::cos(noisy_yaw / 2.0);

        // 5. Assign fixed covariance matrices.
        out.pose.covariance  = pose_cov_;
        out.twist.covariance = twist_cov_;

        // 6. Publish processed odometry.
        pub_->publish(out);

        // 7. Publish encoder-only TF (bypasses EKF — see constructor comment).
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header          = out.header;
        tf_msg.header.frame_id = "odom_enc";
        tf_msg.child_frame_id  = "base_footprint_enc";
        tf_msg.transform.translation.x = out.pose.pose.position.x;
        tf_msg.transform.translation.y = out.pose.pose.position.y;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation      = out.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // Noise model parameters
    double lin_ratio_, ang_ratio_, drift_rate_;

    // Accumulated yaw drift state (random walk — unbounded by design)
    double yaw_drift_ = 0.0;

    // Previous position for distance-based drift computation
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool   first_msg_ = true;

    // RNG
    std::mt19937 gen_;
    std::normal_distribution<double> dist_norm_;

    // Fixed covariance matrices (6×6 row-major diagonal)
    std::array<double, 36> pose_cov_;
    std::array<double, 36> twist_cov_;

    std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyOdomNode>());
    rclcpp::shutdown();
    return 0;
}
