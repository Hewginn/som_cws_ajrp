#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <array>
#include <cmath>

using std::placeholders::_1;

class DistanceCalculator : public rclcpp::Node
{
public:
    DistanceCalculator() : Node("distance_calculator")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "coordinates", 10, std::bind(&DistanceCalculator::GetCoordinate, this, _1)
        );
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("distances", 10);
        RCLCPP_INFO(this->get_logger(), "Distance Calculator Node has been started.");
    }

private:
    void GetCoordinate(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        size_t number_of_dimensions = msg->layout.dim[1].stride;
        size_t number_of_coordinates = msg->layout.dim[0].size;

        std::vector<std::vector<float>> coordinates;

        for (size_t i = 0; i < number_of_coordinates; i++)
        {
            float x = msg->data[i * number_of_dimensions];
            float y = msg->data[i * number_of_dimensions + 1];
            float z = msg->data[i * number_of_dimensions + 2];
            std::vector<float> coordinate = {x, y, z};
            coordinates.push_back(coordinate);
        }
        PublishDistances(coordinates);
    }

    void PrintDistances(std::vector<std::vector<float>> distances){
        RCLCPP_INFO(this->get_logger(), "Published distances:");
        for(int i = 0; i < (int)distances.size(); i++){
            std::string row = "\t[";
            for(int j = 0; j < (int)distances.size(); j++){
                row += std::to_string(distances[i][j]);
                if(j != (int)distances.size() - 1){
                    row += "\t";
                }
            }
            row += "]\t";
            RCLCPP_INFO(this->get_logger(), "%s", row.c_str());
        }
    }

    std::vector<std::vector<float>> CalculateDistances(std::vector<std::vector<float>> coordinates){

        std::vector<std::vector<float>> distances;

        for(int i = 0; i < (int)coordinates.size(); i++){
            std::vector<float> row;
            for(int j = 0; j < (int)coordinates.size(); j++){
                row.push_back(sqrt(
                    (coordinates[i][0]-coordinates[j][0]) * (coordinates[i][0]-coordinates[j][0]) +
                    (coordinates[i][1]-coordinates[j][1]) * (coordinates[i][1]-coordinates[j][1]) +
                    (coordinates[i][2]-coordinates[j][2]) * (coordinates[i][2]-coordinates[j][2])
                ));
            }
            distances.push_back(row);
        }

        return distances;
    }

    

    void PublishDistances(std::vector<std::vector<float>> coordinates){

        std::vector<std::vector<float>> distances = CalculateDistances(coordinates);


        auto message = std_msgs::msg::Float32MultiArray();

        // Setting up layout
        
        message.layout.dim.resize(2);
        message.layout.dim[0].label = "coordinates";
        message.layout.dim[0].size = distances.size();
        message.layout.dim[0].stride = distances.size() * distances.size();

        message.layout.dim[1].label = "distances_of_coordinate";
        message.layout.dim[1].size = distances.size();
        message.layout.dim[1].stride = distances.size();

        std::vector<float> distances_vector;

        for(int i = 0; i < (int)distances.size(); i++){
            for(int j = 0; j < (int)distances.size(); j++){
                distances_vector.push_back(distances[i][j]);
            }
        }

        PrintDistances(distances);

        message.data = distances_vector;

        publisher_->publish(message);
         
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistanceCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
