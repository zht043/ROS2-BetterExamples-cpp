#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using rclcpp::TimerBase;

class Node1 : public rclcpp::Node {
public:
    Node1() : Node("Node1") {
        /* create a timer that calls the timer_callback func for every 0.5 seconds */
        timer = create_wall_timer(
            0.5s, 
            std::bind(&Node1::timer_callback, this)
        );
    }
private:
    void timer_callback();
    std::shared_ptr<TimerBase> timer;
};


void Node1::timer_callback() {
    RCLCPP_INFO(get_logger(), "Hello World!");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<Node1>()
    );
    
    rclcpp::shutdown();
    return 0;
}