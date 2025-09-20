#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <memory>
#include <string>

/*
Mk1 of timestamper
  Standardized every Node (Except AMS , as Float32 MultiArray (7 degit precision))
  AMS will get Battery State Message
      (with Cell voltage as dynamic array , will includes all 80 cells , partition at ten by car design)
  Having separate subscriber of about x topic , each topic contain all sensor data for one node
  Having ROS2 Parameter in the microROS workspace    
Mk2 of timestamper
  Move the host side scipt such as this one back to the ROS2 WS, 
  microROS_WS , I'll try firmware workspace ,see if I can work with it
  if all goes a success, we can make both workspace compile custome msg into its own build file,
  soooo

MK3 of timestamper
  subscribe to only Rx and Tx topics , we do receiving and sending there
  Another custom code is create to parse CAN data into a simple frame structure , stamp every frame with unix timestamp
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
  std::string canRxtopic = "/CAN/can0/receive";
  // std::string canRxtopic = "receive";
  enum MessageID { AMS = 0x00 , BAMO = 0x01, FRONT = 0x02, REAR = 0x03 };
  // otherwise than this, we will set a condition based on bp16 communication agreement that we needs
  rclcpp::TimerBase::SharedPtr timer_task1;
  rclcpp::TimerBase::SharedPtr timer_task2;
  size_t count_;
  
  // Create publisher Node pointer of type CAN Frame
  
  rclcpp::Subscription<Frame>::SharedPtr sub;
  rclcpp::Publisher<Frame>::SharedPtr pub_ams;
  rclcpp::Publisher<Frame>::SharedPtr pub_bamo;
  rclcpp::Publisher<Frame>::SharedPtr pub_front;
  rclcpp::Publisher<Frame>::SharedPtr pub_rear;

  // rclcpp::Publisher<Frame>::SharedPtr bmu;

  // Define can frame
  Frame last_ams_msg;
  Frame last_bamo_msg;
  Frame last_front_msg;
  Frame last_rear_msg;

  // Later this should be std vector of pointer to frame or just frame
    // might optimize later
  // Frame bmu1;
  // Frame bmu2;
  // Frame bmu3;
  // Frame bmu4;
  // Frame bmu5;
  // Frame bmu6;
  // Frame bmu7;
  // Frame bmu8;

  // There should be only 4 message , but the worrysome case is BMU internal bus that I supppose to be a separate communication
  // channel, since it isn't

public:
  // class variable here
  TimestamperWrapper(): Node("TimestamperWrapper"), count_(0)
  {    
    // The subscribed message comes from ROS2socketcan, provide template messageT name
    // call sub_cb lambda expression above when subscribed topics
    sub = this->create_subscription<Frame>(canRxtopic, 10,
      std::bind(&TimestamperWrapper::sub_sbRx, this, std::placeholders::_1));

    pub_ams = this->create_publisher<Frame>("AMS_stamped",10);
    pub_bamo = this->create_publisher<Frame>("BAMO_stamped",10);
    pub_front = this->create_publisher<Frame>("Front_stamped",10);
    pub_rear = this->create_publisher<Frame>("Rear_stamped",10);
    
    // bmu = this->create_publisher<Frame>("BMU",10);

    // Timer for publishing callback
    // Timer for debugger callback
    timer_task1 = this->create_wall_timer(TIMER_PUBLISH_PERIOD_MS, std::bind(&TimestamperWrapper::publish_topic, this));
    timer_task2 = this->create_wall_timer(TIMER_PUBLISH_PERIOD_MS, std::bind(&TimestamperWrapper::debugger,this));
  };

  void sub_sbRx(const Frame::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(),"I know you got ");
    int64_t now = this->get_clock()->now().nanoseconds();
    // welp still not sure how this would work
    MessageID ID = static_cast<MessageID>(msg->id);
    
    switch (ID) {
      // assign our own CAN frame to each recivied ID 
      case AMS: 
        last_ams_msg = *msg; last_ams_msg.header.stamp.nanosec = now;     break;
      case BAMO: 
        last_bamo_msg = *msg; last_bamo_msg.header.stamp.nanosec = now;   break;
      case FRONT: 
        last_front_msg = *msg; last_front_msg.header.stamp.nanosec = now; break;
      case REAR: 
        last_rear_msg = *msg; last_rear_msg.header.stamp.nanosec = now;   break;
      
      // if switch case doesn't comply with enum -> it must be BMU internal bus
      default:
        ;
    }
  }

  void publish_topic(){
    pub_ams->publish(last_ams_msg);
    pub_bamo->publish(last_bamo_msg);
    pub_front->publish(last_front_msg);
    pub_rear->publish(last_rear_msg);
  }

  void debugger(){
    RCLCPP_INFO(this->get_logger(),"1.AMS CAN Frame: %03x", last_ams_msg.id);
    RCLCPP_INFO(this->get_logger(),"2.BAMO CAN Frame: %03x", last_bamo_msg.id);
    RCLCPP_INFO(this->get_logger(),"3.FRONT Frame: %03x", last_front_msg.id);
    RCLCPP_INFO(this->get_logger(),"4.REAR Frame: %03x", last_rear_msg.id);
    // Debug the messsage inside (won't translate)
  }
};
/****************************************
Main
****************************************/

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  // Create a shared pointer to a node object
  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared();
  // Somehow this works , Node is subclass of rclcpp , but isn't Tims
  // std::shared_ptr<TimestamperWrapper::Node> node = TimestamperWrapper::Node::make_shared();
  auto node = std::make_shared<TimestamperWrapper>();
  // Spin the node (or perform other operations)
  rclcpp::spin(node);
  // When 'node' goes out of scope, its destructor is implicitly called.
  // This handles the cleanup of the ROS 2 node.
  rclcpp::shutdown(); // Shutdown the ROS 2 context


  return 0;
}