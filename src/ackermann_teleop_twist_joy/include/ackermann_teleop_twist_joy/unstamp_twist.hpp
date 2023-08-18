#ifndef UNSTAMP_TWIST_H
#define UNSTAMP_TWIST_H

namespace unstamp_twist
{
    class __attribute__((visibility("default"))) UnstampTwist : public rclcpp::Node
    {
    public:
        explicit UnstampTwist(const rclcpp::NodeOptions& options);

    private:
        rclcpp::Subscriber<geometry_msgs::msg::TwistStamped> twist_stamped_topic;
        rclcpp::Publisher<geometry_msgs::msg::Twist> twist_unstamped_topic;
        OnSetParametersCallbackHandle::SharedPtr callback_handle;    
    }
}

#endif  // UNSTAMP_TWIST_H
