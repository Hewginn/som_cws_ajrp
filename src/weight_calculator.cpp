#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <array>
#include <cmath>

using std::placeholders::_1;

class WeightCalculator : public rclcpp::Node
{
public:
    WeightCalculator() : Node("weight_calculator")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "coordinates", 10, std::bind(&WeightCalculator::topic_callback, this, _1)
        );
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("weights", 10);
        RCLCPP_INFO(this->get_logger(), "Weight Calculator Node has been started.");
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        size_t number_of_dimensions = msg->layout.dim[1].stride;
        size_t number_of_coordinates = msg->layout.dim[0].size;

        RCLCPP_INFO(this->get_logger(), "Received coordinates:");

        std::vector<std::vector<float>> coordinates;

        for (size_t i = 0; i < number_of_coordinates; i++)
        {
            float x = msg->data[i * number_of_dimensions];
            float y = msg->data[i * number_of_dimensions + 1];
            float z = msg->data[i * number_of_dimensions + 2];
            std::vector<float> coordinate = {x, y, z};
            coordinates.push_back(coordinate);
            RCLCPP_INFO(this->get_logger(), "  Coordinate %ld: [x = %f, y = %f, z = %f]", i + 1, coordinates[i][0], coordinates[i][1], coordinates[i][2]);
        }

        publish_weights(coordinates);
    }

    std::vector<std::vector<float>> weights_gen(std::vector<std::vector<float>> coordinates){

        std::vector<std::vector<float>> weights;

        for(int i = 0; i < (int)coordinates.size(); i++){
            std::vector<float> row;
            for(int j = 0; j < (int)coordinates.size(); j++){
                row.push_back(sqrt(
                    (coordinates[i][0]-coordinates[j][0]) * (coordinates[i][0]-coordinates[j][0]) +
                    (coordinates[i][1]-coordinates[j][1]) * (coordinates[i][1]-coordinates[j][1]) +
                    (coordinates[i][2]-coordinates[j][2]) * (coordinates[i][2]-coordinates[j][2])
                ));
            }
            weights.push_back(row);
        }

        return weights;
    }

    

    void publish_weights(std::vector<std::vector<float>> coordinates){

        std::vector<std::vector<float>> weights = weights_gen(coordinates);

        auto message = std_msgs::msg::Float32MultiArray();

        // Setting up layout
        
        message.layout.dim.resize(2);
        message.layout.dim[0].label = "coordinates";
        message.layout.dim[0].size = weights.size();
        message.layout.dim[0].stride = weights.size() * weights.size();

        message.layout.dim[1].label = "weights_of_coordinate";
        message.layout.dim[1].size = weights.size();
        message.layout.dim[1].stride = weights.size();

        std::vector<float> weight_vector;

        for(int i = 0; i < (int)weights.size(); i++){
            for(int j = 0; j < (int)weights.size(); j++){
                weight_vector.push_back(weights[i][j]);
            }
        }

        message.data = weight_vector;

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published weights: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
            message.data[0], message.data[1], message.data[2],
            message.data[3], message.data[4], message.data[5],
            message.data[6], message.data[7], message.data[8]);
            
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeightCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
