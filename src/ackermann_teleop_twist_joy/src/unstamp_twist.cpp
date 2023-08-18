UnstampTwist::UnstampTwist(const rclcpp::NodeOptions& oprions) : Node("unstamp_twist", options)
{
    std::string twist_stamped_topic = this->declare_parameter("twist_stamped_topic", "/cmd_vel");
    std::string twist_unstamped_topic = this->declare_parameter("twist_unstamped_topic", "/cmd_vel_unstamped");
    
    this->twist_stamped_topic = 
    this->create_subscription<geometry_msgs::msg::TwistStamped>(twist_stamped_topic, rclcpp::QoS(10),
        std::bind(&UnstampTwist::cmd_vel_callback, this, std::placeholders::_1))
    
    ROS_INFO_NAMED("UnstampTwist", "Stamped topic: %s", twist_stamped_topic.c_str());
    ROS_INFO_NAMED("UnstampTwist", "Unstamped topic: %s", twist_unstamped_topic.c_str());



    auto param_callback =
    [this](std::vector<rclcpp::Parameter> parameters)
    {
        static std::set<std::string> strparams = {"twist_stamped_topic", "twist_unstamped_topic"};
        auto result = rcl_interfaces::msg::SetParametersResult();

        // Loop to check if changed parameters are of expected data type
        for (const auto& parameter : parameters)
        {
            if (strparams.count(parameter.get_name() == 1))
            {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
                {
                    result.reason = "Only string values can be set for '" + parameter.get_name() + "'.";
                    RCLCPP_WARN(this->get_logger(), result.reason.c_str());
                    result.successful = false;
                    return result;
                }
            }
        }
    
        // Loop to assign changed parameters to the member variables
        for (const auto & parameter : parameters)
        {
            if (parameter.get_name() == "twist_stamped_topic")
            {
                std::string twist_stamped_topic = parameter.get_value<rclcpp::PARAMETER_STRING>();

            }
            if (parameter.get_name() == "twist_unstamped_topic")
                this->twist_unstamped_topic = parameter.get_value<rclcpp::PARAMETER_STRING>();
        }
    }

    callback_handle = this->add_on_set_parameters_callback(param_callback);
}



RCLCPP_COMPONENTS_REGISTER_NODE(ackermann_teleop_twist_joy::AckermannTeleopTwistJoy)
