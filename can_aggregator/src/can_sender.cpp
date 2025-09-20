#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <memory>
#include <string>

/*
Mk1 of transmitter
  aggregator .yaml to launch this with stamper
  There are no condition or control logic 

Mk 2 Add control logic
  Decided by ros2 parameter passed from stamper
*/

/****************************************
Node class and can frame packing methods
****************************************/

using namespace can_msgs::msg;
using namespace std::chrono;
#define TIMER_PUBLISH_PERIOD_MS 200ms

class TimestamperWrapper : public rclcpp::Node {
// Inherit rclcpp::Node create a publisher node 
private:
  //  Topic list
  std::string canTxtopic = "/CAN/can0/transmit";

  enum MessageID { AMS = 0x00 , BAMO = 0x01, FRONT = 0x02, REAR = 0x03 };
  // otherwise than this, we will set a condition based on bp16 communication agreement that we needs

  rclcpp::TimerBase::SharedPtr timer_task1;
  rclcpp::TimerBase::SharedPtr timer_task2;
  size_t count_;
  
  // Create publisher Node pointer of type CAN Frame,  
  rclcpp::Publisher<Frame>::SharedPtr can_tx;

  // Define can frame
  Frame my_msg;

public:
  // class variable here
  TimestamperWrapper(): Node("TimestamperWrapper"), count_(0)
  {    
    // The subscribed message comes from ROS2socketcan, provide template messageT name
    can_tx = this->create_publisher<Frame>(canTxtopic,10);

    /*
    Control logics and condition recevied from ros2 parameter //
    */
    
    // Timer for publishing callback
    timer_task1 = this->create_wall_timer(TIMER_PUBLISH_PERIOD_MS, std::bind(&TimestamperWrapper::publish_topic, this));
  };

  void packCANFrame(const Frame::SharedPtr msg, uint16_t standardID , uint8_t dlc, uint8_t* Data){
    int64_t now = this->get_clock()->now().nanoseconds();
    msg->id = standardID;
    msg->header.stamp.nanosec = now;
    msg->dlc = dlc;
    std::copy(Data, Data + dlc, msg->data.begin()); // Repackage data from
  }

  void publish_topic(){
    can_tx->publish(my_msg);
  }

};
/****************************************
Main
****************************************/

int main(int argc, char * argv[]) {

  

  rclcpp::init(argc, argv);

  // --- Uncomment below to start using this node

  /*
  // Create a shared pointer to a node object
  auto node = std::make_shared<TimestamperWrapper>();
  // Spin the node (or perform other operations)
  rclcpp::spin(node);
  rclcpp::shutdown(); // Shutdown the ROS 2 context
  */

  return 0;
}