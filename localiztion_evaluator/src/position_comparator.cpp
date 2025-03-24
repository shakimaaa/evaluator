#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gps_msg/msg/global_position_int.hpp" 
#include "nav_msgs/msg/path.hpp" 
#include <GeographicLib/LocalCartesian.hpp>
#include <cmath>

class PositionErrorNode : public rclcpp::Node {
public:
    PositionErrorNode() : Node("position_error_node") {
        // 设定参考点（起飞点的 GPS 经纬度）
        double ref_lat = 37.7749;  // 需要修改为实际基准点
        double ref_lon = -122.4194;
        double ref_alt = 0.0;
        geo_converter_.Reset(ref_lat, ref_lon, ref_alt);

        // 订阅 MAVLink 的自定义 GLOBAL_POSITION_INT
        mavlink_sub_ = this->create_subscription<gps_msg::msg::GlobalPositionInt>(
            "/mavlink/global_position_int", 10,
            std::bind(&PositionErrorNode::mavlink_callback, this, std::placeholders::_1));

        // 订阅 SLAM 定位的 PoseStamped
        slam_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/pose", 10,
            std::bind(&PositionErrorNode::slam_callback, this, std::placeholders::_1));

        // 初始化轨迹发布器
        mavlink_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mavlink_path", 10);
        slam_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/slam_path", 10);

        // 初始化轨迹消息
        mavlink_path_.header.frame_id = "map";  // 假设坐标系为map
        slam_path_.header.frame_id = "map";
    }

private:
    rclcpp::Subscription<gps_msg::msg::GlobalPositionInt>::SharedPtr mavlink_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_sub_;

    gps_msg::msg::GlobalPositionInt last_mavlink_msg_;
    geometry_msgs::msg::PoseStamped last_slam_msg_;
    bool has_mavlink_ = false, has_slam_ = false;

    // 添加轨迹发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mavlink_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;

    // 轨迹消息
    nav_msgs::msg::Path mavlink_path_;
    nav_msgs::msg::Path slam_path_;

    // GeographicLib 用于 GPS 经纬度 -> ENU 坐标转换
    GeographicLib::LocalCartesian geo_converter_;

    void mavlink_callback(const gps_msg::msg::GlobalPositionInt::SharedPtr msg) {
        last_mavlink_msg_ = *msg;
        has_mavlink_ = true;
        compute_error();
    }

    void slam_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_slam_msg_ = *msg;
        has_slam_ = true;
        compute_error();
    }

    void compute_error() {
        if (has_mavlink_ && has_slam_) {
            // 将 MAVLink 经纬度转换为 ENU 坐标
            double mav_x, mav_y, mav_z;
            geo_converter_.Forward(last_mavlink_msg_.lat / 1e7,  // lat 转换为度
                                   last_mavlink_msg_.lon / 1e7,  // lon 转换为度
                                   last_mavlink_msg_.alt / 1000.0,  // alt 转换为米
                                   mav_x, mav_y, mav_z);

            // 计算 SLAM 与 GPS 的位置误差
            double dx = last_slam_msg_.pose.position.x - mav_x;
            double dy = last_slam_msg_.pose.position.y - mav_y;
            double dz = last_slam_msg_.pose.position.z - mav_z;
            double error = std::sqrt(dx * dx + dy * dy + dz * dz);

            RCLCPP_INFO(this->get_logger(), "Position Error: %.3f meters", error);

            // 发布 MAVLink 轨迹
            geometry_msgs::msg::PoseStamped mav_pose;
            mav_pose.header.stamp = this->now();
            mav_pose.header.frame_id = "map";
            mav_pose.pose.position.x = mav_x;
            mav_pose.pose.position.y = mav_y;
            mav_pose.pose.position.z = mav_z;
            mav_pose.pose.orientation.w = 1.0;  // 无旋转
            mavlink_path_.poses.push_back(mav_pose);
            mavlink_path_.header.stamp = this->now();
            mavlink_path_pub_->publish(mavlink_path_);

            // 发布 SLAM 轨迹
            geometry_msgs::msg::PoseStamped slam_pose = last_slam_msg_;
            slam_pose.header.stamp = this->now();
            slam_pose.header.frame_id = "map";
            slam_path_.poses.push_back(slam_pose);
            slam_path_.header.stamp = this->now();
            slam_path_pub_->publish(slam_path_);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionErrorNode>());
    rclcpp::shutdown();
    return 0;
}