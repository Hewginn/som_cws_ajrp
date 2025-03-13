
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cfloat>
#include "std_msgs/msg/int32_multi_array.hpp"
#include <std_msgs/msg/detail/int32_multi_array__struct.hpp>
#include <climits>

using std::placeholders::_1;

class TspSolver : public rclcpp::Node
{
  public:
    TspSolver() : Node("tsp_solver")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "distances", 10, std::bind(&TspSolver::GetDistances, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("best_order", 10);
      RCLCPP_INFO(this->get_logger(), "Traveling Sales Man Solver Node has been started.");
    }

  private:
    void GetDistances(const std_msgs::msg::Float32MultiArray::SharedPtr msg){

      size_t dimensions = msg->layout.dim[1].stride;

      std::vector<std::vector<float>> distances;

      for (size_t i = 0; i < dimensions; i++){
        std::vector<float> row;
        for(size_t j = 0; j < dimensions; j++){
          row.push_back(msg->data[i*dimensions + j]);
        }
        distances.push_back(row);
      }

      PublishBestOrder(distances);
    }

    std::vector<int> TspFull(std::vector<std::vector<float>> cost) {

      // Number of nodes
      int num_nodes = cost.size();
      std::vector<int> nodes;
  
      // Initialize the nodes excluding the fixed 
      // starting point (node 0)
      for (int i = 1; i < num_nodes; i++){
          nodes.push_back(i);
      }

      float min_cost = FLT_MAX;

      std::vector<int> best_permutation;
  
      // Generate all permutations of the remaining nodes
      do {
          float curr_cost = 0;
  
          // Start from node 1
          int curr_node = 0;
  
          // Calculate the cost of the current permutation
          for (int i = 0; i < num_nodes - 1; i++) {
              curr_cost += cost[curr_node][nodes[i]];
              curr_node = nodes[i];
          }

  
          // Add the cost to return to the starting node
          curr_cost += cost[curr_node][0];
  
        // Update the minimum cost if the current cost 
        // is lower
        if(min_cost > curr_cost){
          min_cost = curr_cost;
          best_permutation = nodes;
        }
  
      } while (next_permutation(nodes.begin(), nodes.end()));

      return best_permutation;
  }

  void PrintSolution(std::vector<int> solution){
    RCLCPP_INFO(this->get_logger(), "Best order:");
    std::string out = "\tStart (0. Coordinate)\t->\t";

    for(int i = 0; i < (int)solution.size(); i++){
      out += std::to_string(solution[i]) + ". Coordinate";
      if(i != (int)solution.size() - 1){
        out += "\t->\t";
      }
    }

    RCLCPP_INFO(this->get_logger(), "%s", out.c_str());
  }

  void PublishBestOrder(std::vector<std::vector<float>> cost){
    std::vector<int> best_order = TspFull(cost);
    auto message = std_msgs::msg::Int32MultiArray();
    message.data = best_order;
    publisher_->publish(message);
    PrintSolution(best_order);
  }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TspSolver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}