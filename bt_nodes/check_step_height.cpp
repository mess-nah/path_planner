#include <behavior_tree_cpp_v3/action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include  <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>


class CheckStepHeight public: BT::SyncActionNode
{
    public:
        CheckStepHeight(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("check_step_height");
            climb_pub_=node->create_publisher<std_msgs::msg::String("/climb_cmd", 10);
            spin_thread_=std::thread([this](){
                rclcpp::spin(node);
            });
        }

        ~CheckStepHeight()
        {
            rclcpp::shutdown();
            if (spin_thread_.joinable()) spin_thread_.join();

        }
        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<int>("current_node"),
                BT::InputPort<int.("next_node")
            };
        }
        BT::NodeStatus tick() override
        {
            int current_node, next_node;
            if (!getInput("current_node", current_node)|| 
                    (!getInput("next_node", next_node)))
            {
                RCLCPP_ERROR(node_->get_logger(), "Missing node inputs");
                return BT::NodeStatus::FAILURE;
            }
            std::unordered_map<int,int> node_height = 
            {
            {1,40},{2,20},{3,40},{4,60},
            {5,40},{6,20},{7,40},{8,60},
            {9,40},{10,20},{11,40},{12,20}
            };

            int height_curr = node_height[current_node];
            int height_next = node_height[next_node];
            int height_diff = height_next - height_curr;

            std_msgs::msg::String msg;

            if (height_diff == 20)
            {
                msg.data = "UP";
                climb_pub_->publish(msg);
                RCLCPP_INFO(node_->get_logger(), "Climbing UP (20cm)");
                return BT::NodeStatus::SUCCESS;
            }
            else if (height_diff == -20)
            {
                msg.data = "DOWN";
                climb_pub_->publish(msg);
                RCLCPP_INFO(node_->get_logger(), "Climbing DOWN (-20cm)");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "No step, continue");
                return BT::NodeStatus::SUCCESS;
            }


        }
        private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr climb_pub_;
        std::thread spin_thread_;
};
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<CheckStepHeight>("CheckStepHeight");
}
