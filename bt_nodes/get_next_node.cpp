#include <behavior_tree_cpp_v3/action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class GetNextNode : public : BT ::SyncActionNode
{
    public:
        GetNextNode(const std::String& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("get_next_node");
        }
        ~GetNextNode()
        {
            rclcpp::shutdown();
        }
         static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<int>>("path"),
                BT::InputPort<int>("current_index"),
                BT::OutputPort<int>("next_node"),
                BT::OutputPort<int>("next_index")
            };
        }
         BT::NodeStatus tick() override
        {
            std::vector<int> path;
            int current_index;

            if (!getInput("path", path) || !getInput("current_index", current_index))
            {
                RCLCPP_ERROR(node_->get_logger(), "[GetNextNode] Missing inputs");
                return BT::NodeStatus::FAILURE;
            }

            if (current_index >= (int)path.size() - 1)
            {
                RCLCPP_INFO(node_->get_logger(), "[GetNextNode] Reached final goal");
                return BT::NodeStatus::FAILURE;
            }

            int next_node = path[current_index + 1];
            int next_index = current_index + 1;

            setOutput("next_node", next_node);
            setOutput("next_index", next_index);

            RCLCPP_INFO(node_->get_logger(), "[GetNextNode] Current: %d, Next: %d", path[current_index], next_node);

            return BT::NodeStatus::SUCCESS;
        }

    private:
            rclcpp::Node::SharedPtr node_;

};
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<GetNextNode>("GetNextNode");
}

