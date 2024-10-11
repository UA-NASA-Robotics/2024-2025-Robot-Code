//default includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "whatever message type needed from "/human_input" and "/autonomy_input" topics

using std::placeholders::_1;

class PriorityNode : public rclcpp::Node
{
public:
    PriorityNode(): Node("priority_node")
    {
        //Node subscribes to "/human_input"
        //calls the human_input_callback function 
        //after recieving a /msg as a String
        human_input_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/human_input", 10, std::bind(&PriorityNode::human_input_callback, this, _1));

        //Node subscribes to "/autonomy_input"
        //calls the autonomy_input_callback function 
        //after recieving a /msg as a String
        autonomy_input_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/autonomy_input", 10, std::bind(&PriorityNode::autonomy_input_callback, this, _1));

        //Node publishes to "/desired_movement" 
        //it does this within the publish_desired_movement function 
        //which is called explicitly in each callback
        desired_movement_publisher_ = this->create_publisher<std_msgs::msg::String>("/desired_movement", 10);

        //helps determine which /msg should be published
        //if true, it blocks /msgs from "/human_input"
        //it is always false unless an "autonomy_input" msgs is recieved
        use_autonomy_input_ = false;
    }

private:
    /**
     * @brief Passes /msg from "/human_input" to "desired_movement" when a msg is recieved.
     * 
     * @details
     * This only happens if there is no /msgs from "/autonomy_input"
     * The msg type can be changed later to better fit inplementation with
     * RS_package and Control_Que node
     * 
     * @param msg a String to be passed into the function, see details above for info.
     */
    void human_input_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!use_autonomy_input_)
        {
            publish_desired_movement(msg->data);
        }
    }

    /**
     * @brief Passes /msg from "/autonomy_input" to "desired_movement" when a msg is recieved.
     * 
     * @details
     * This takes publishing priority over "/human_input" and will block any messages from sending
     * The msg type can be changed later to better fit inplementation with
     * RS_package and Control_Que node
     * 
     * @remark it is unknown if the blocking causes any "/human_input" msg to be stored in a buffer
     * or simply discarded. I believe any msg is simply discarded.
     * 
     * @param msg a String to be passed into the function, see details above for info.
     */
    void autonomy_input_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        use_autonomy_input_ = true;
        publish_desired_movement(msg->data);
        use_autonomy_input_ = false;
    }

    /**
     * @brief Publishes /msg to "/desired_input"
     * 
     * @details
     * This function is called in the human and autonomy input_callback functions
     * it takes whatever msg is passed in and publishes it to "/desired_movement"
     * to be parsed and que'd by a node called control_que
     * 
     * @param movement a String to be sent. Type can (and should) be modified to suit needs.
     */
    void publish_desired_movement(const std::string &movement)
    {
        auto message = std_msgs::msg::String();
        message.data = movement;
        desired_movement_publisher_->publish(message);
    }

    //variables -- 3 shared pointers and 1 global boolean
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr human_input_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr autonomy_input_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr desired_movement_publisher_;
    bool use_autonomy_input_;
};

/*main function is "default" ROS2 node runner function.
 *initializes a node, and spins (keeps alive) until shutdown
 */then exits.
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PriorityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
