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
            std::chrono::seconds(5),
            std::bind(&CoordinatePublisher::PublishMessage, this)
        );
        RCLCPP_INFO(this->get_logger(), "Coordinate Publisher Node has been started.");
    }

    void PublishMessage(){
        auto message = GenerateMessage();
        publisher_->publish(message);
        PrintCoordinates(message);
    }

private:

    void PrintCoordinates(std_msgs::msg::Float32MultiArray message){
        RCLCPP_INFO(this->get_logger(), "Published coordinates:");
        for(int i = 0; i < (int)message.layout.dim[0].size; i++){
            RCLCPP_INFO(this->get_logger(),
            "\t%d Coordinate: [%f, %f, %f]", i+1, message.data[i*3], message.data[i*3+1], message.data[i*3+2]);
        }
    }

    std_msgs::msg::Float32MultiArray GenerateMessage()
    {
        auto message = std_msgs::msg::Float32MultiArray();

        int num_coordinates = rand() % 8 + 3;

        // Setting up layout
        message.layout.dim.resize(2);
        message.layout.dim[0].label = "coordinate_set";
        message.layout.dim[0].size = num_coordinates;  // number of coordinates
        message.layout.dim[0].stride = num_coordinates * 3; // number of values (stride / size = number of dimensions)

        message.layout.dim[1].label = "coordinate";
        message.layout.dim[1].size = 3;  // number of dimensions
        message.layout.dim[1].stride = 3; // number of dimensions

        std::vector<float> generated_coordinates;

        generated_coordinates.push_back(0);
        generated_coordinates.push_back(0);
        generated_coordinates.push_back(0);

        for(int i = 3; i < num_coordinates * 3; i++){
            generated_coordinates.push_back(rand() % 10 - rand() % 10);
        }

        message.data = generated_coordinates;

        return message;
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