 
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <octomap/octomap.h>

class OctomapAccuracyEvaluator : public rclcpp::Node
{
public:
    OctomapAccuracyEvaluator() : Node("octomap_accuracy_evaluator"), score_(0)
    {
        // 直接定义障碍物的位置和大小（每个障碍物为 0.6m 边长的立方体）
        reality_obstacles_ = {
            {1.0, 2.0, 0.5},  // 障碍物 1 中心坐标 (x, y, z)
            {3.0, 4.0, 1.5},  // 障碍物 2
            {5.0, 6.0, 2.0},  // 障碍物 3
        };

        // 订阅 Octomap 消息
        octomap_subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap", 10, std::bind(&OctomapAccuracyEvaluator::octomap_callback, this, std::placeholders::_1));
    }

private:
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        // 将二进制数据保存到临时文件
        std::string filename = "/tmp/octomap_data.bt";
        std::ofstream out_file(filename, std::ios::binary);
        if (out_file.is_open())
        {
            out_file.write(reinterpret_cast<const char *>(msg->data.data()), msg->data.size());
            out_file.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write Octomap data to temporary file.");
            return;
        }

        // 读取临时文件中的 Octomap 数据
        octomap::OcTree octree(0.1); // 假设分辨率为 0.1
        if (!octree.readBinary(filename))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read Octomap from file.");
            return;
        }

        // 对于每个现实中的障碍物，计算与 Octomap 中的障碍物的误差
        for (const auto &reality_obstacle : reality_obstacles_)
        {
            double x_real = reality_obstacle[0];
            double y_real = reality_obstacle[1];
            double z_real = reality_obstacle[2];

            // 假设每个障碍物是一个 0.6m 的正方体，计算立方体的边界
            double half_size = 0.3;  // 0.6m 边长的正方体，半边为 0.3m

            // 检查 Octomap 中是否存在该区域的障碍物
            bool is_obstacle_detected = false;

            // 遍历立方体的 8 个角点并检查是否有障碍物
            for (double dx = -half_size; dx <= half_size; dx += half_size)
            {
                for (double dy = -half_size; dy <= half_size; dy += half_size)
                {
                    for (double dz = -half_size; dz <= half_size; dz += half_size)
                    {
                        octomap::point3d point(x_real + dx, y_real + dy, z_real + dz);
                        if (octree.search(point))
                        {
                            is_obstacle_detected = true;
                            break;
                        }
                    }
                    if (is_obstacle_detected) break;
                }
                if (is_obstacle_detected) break;
            }

            // 如果检测到障碍物，并且误差小于阈值，则加分
            if (is_obstacle_detected)
            {
                score_++;
            }
            else
            {
                score_--;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Current accuracy score: %d", score_);
    }

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscription_;
    std::vector<std::array<double, 3>> reality_obstacles_; // 存储现实中的障碍物 (x, y, z)
    int score_; // 准确度评分
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapAccuracyEvaluator>());
    rclcpp::shutdown();
    return 0;
}