#include <behavior_tree_cpp_v3/action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.cpp>

class TurnToTarget : public: BT::SyncActionNode
{
    public:
        TurnToTarget(const std::string& name, const BT::NodeConfiguration& config)
        :BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("turn_to_target_bt");
            cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
         BT::NodeStatus tick() override
        {

            int diff;
            if (!getInput<int>("diff" ,diff))
            {
                throw BT::RunTimeError("missing required input [diff]");
            }
            double angle_deg = 0.0;
            if(diff == -1) angle_deg = 90;
            else if(diff == 1) angle_deg = -90;
            else angle_deg = 0;

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = angle_deg * 3.14159 / 180.0;

            cmd_pub_->publish(cmd);
            return BT::NodeStatus::SUCCESS;

        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::Sharedptr cmd_pub_;


};

BT_REGISTER_NODES(factory)
{
    factoryregisterNodeType<TurnToTarget>("TurnToTarget");
}