#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class StopRobot : public BT::SyncActionNode
{
public:
    StopRobot(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_stop_robot");
        cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // keep ROS spinning so publishers work
        spin_thread_ = std::thread([this]() {
            rclcpp::spin(node_);
        });
    }

    ~StopRobot()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;

        cmd_pub_->publish(stop_cmd);

        RCLCPP_INFO(node_->get_logger(), "[StopRobot] Robot stopped.");

        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::thread spin_thread_;
};

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<StopRobot>("StopRobot");
}
