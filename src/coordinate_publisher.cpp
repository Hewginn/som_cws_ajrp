#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;

class CoordinatePublisher : public rclcpp::Node
{
public:
    CoordinatePublisher() : Node("coordinate_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("coordinates", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CoordinatePublisher::publish_coordinates, this)
        );
        RCLCPP_INFO(this->get_logger(), "Coordinate Publisher Node has been started.");
    }

private:
    void publish_coordinates()
    {
        auto message = std_msgs::msg::Float32MultiArray();

        // Setting up layout
        message.layout.dim.resize(2);
        message.layout.dim[0].label = "coordinate_set";
        message.layout.dim[0].size = 3;  // number of coordinates
        message.layout.dim[0].stride = 9; // number of values (stride / size = number of dimensions)

        message.layout.dim[1].label = "coordinate";
        message.layout.dim[1].size = 3;  // number of dimensions
        message.layout.dim[1].stride = 3; // number of dimensions

        message.data = {1.0, 2.0, 3.0,
                        4.0, 5.0, 6.0,
                        7.0, 8.0, 9.0};  

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published coordinates: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                    message.data[0], message.data[1], message.data[2],
                    message.data[3], message.data[4], message.data[5],
                    message.data[6], message.data[7], message.data[8]);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CoordinatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}