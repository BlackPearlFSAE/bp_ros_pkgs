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
  std::string canTx = "/CAN/can0/transmit";
  std::string canRx = "/CAN/can0/receive";

  // in case of reverting back to microROS serial
  std::string TopicAMS = "AMS";
  std::string TopicBAMO = "BAMO";
  std::string TopicFront = "Front";
  std::string TopicRear = "Rear";

  enum TopicType { AMS, BAMO, FRONT, REAR };

  enum MessageID { AMS = 0x00 , BAMO = 0x01, FRONT = 0x02, REAR = 0x03 };
  // otherwise than this, we will set a condition based on bp16 communication agreement that we needs

  rclcpp::TimerBase::SharedPtr timer_task1;
  rclcpp::TimerBase::SharedPtr timer_task2;
  size_t count_;
  
  // Create publisher Node pointer of type CAN Frame,  
  rclcpp::Publisher<Frame>::SharedPtr pub_ams;
  rclcpp::Publisher<Frame>::SharedPtr pub_bamo;
  rclcpp::Publisher<Frame>::SharedPtr pub_front;
  rclcpp::Publisher<Frame>::SharedPtr pub_rear;

  // Define can frame
  Frame last_ams_msg;
  Frame last_bamo_msg;
  Frame last_front_msg;
  Frame last_rear_msg;

  // There should be only 4 message , but the worrysome case is BMU internal bus that I supppose to be a separate communication
  // channel, since it isn't

public:
  // class variable here
  TimestamperWrapper(): Node("TimestamperWrapper"), count_(0)
  {
    auto sub_cb = [this](const Frame::SharedPtr msg, TopicType type) {
      // int64_t now = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
      int64_t now = this->get_clock()->now().nanoseconds();
      switch (type) {
        // assign our own CAN frame to recivied msg 
        case AMS: last_ams_msg = *msg; last_ams_msg.header.stamp.nanosec = now;       break;
        case BAMO: last_bamo_msg = *msg; last_bamo_msg.header.stamp.nanosec = now;    break;
        case FRONT: last_front_msg = *msg; last_front_msg.header.stamp.nanosec = now; break;
        case REAR: last_rear_msg = *msg; last_rear_msg.header.stamp.nanosec = now;    break;
      }
    };

    
    // The subscribed message comes from ROS2socketcan, provide template messageT name
    // call sub_cb lambda expression above when subscribed topics
    this->create_subscription<Frame>(TopicAMS, 10,
      [sub_cb](const Frame::SharedPtr msg) { sub_cb(msg, AMS);});
    this->create_subscription<Frame>(TopicBAMO, 10, 
      [sub_cb](const Frame::SharedPtr msg) { sub_cb(msg, BAMO);});
    this->create_subscription<Frame>(TopicFront, 10,
      [sub_cb](const Frame::SharedPtr msg) { sub_cb(msg, FRONT);});
    this->create_subscription<Frame>(TopicRear, 10,
      [sub_cb](const Frame::SharedPtr msg) { sub_cb(msg, REAR);});
      // std::bind(&TimestamperWrapper::callback_, this, std::placeholders::_4));

    pub_ams = this->create_publisher<Frame>(TopicAMS+"_stamped",10);
    pub_bamo = this->create_publisher<Frame>(TopicBAMO+"_stamped",10);
    pub_front = this->create_publisher<Frame>(TopicFront+"_stamped",10);
    pub_rear = this->create_publisher<Frame>(TopicRear+"_stamped",10);

    // why do I need to bind the callback method to the currently refering object
    timer_task1 = this->create_wall_timer(TIMER_PUBLISH_PERIOD_MS, std::bind(&TimestamperWrapper::publish_topic, this));
    timer_task2 = this->create_wall_timer(TIMER_PUBLISH_PERIOD_MS, std::bind(&TimestamperWrapper::debugger,this));
  };

  void sub_sbTx(const Frame::SharedPtr msg, uint16_t standardID , uint8_t dlc, uint8_t* Data){
    int64_t now = this->get_clock()->now().nanoseconds();
    msg->header.stamp.nanosec = now;
    msg->dlc = dlc;
    // Copy data from Data pointer to the first index of msg->data
    std::copy(Data, Data + dlc, msg->data.begin());
    // for(int i = 0; i< dlc; i++)
    //   msg->data[i] = Data[i];
  }

  void sub_sbRx(const Frame::SharedPtr msg, MessageID ID){
    int64_t now = this->get_clock()->now().nanoseconds();

    switch (ID) {
      // assign our own CAN frame to each recivied ID 
      case AMS: last_ams_msg = *msg; last_ams_msg.header.stamp.nanosec = now;       break;
      case BAMO: last_bamo_msg = *msg; last_bamo_msg.header.stamp.nanosec = now;    break;
      case FRONT: last_front_msg = *msg; last_front_msg.header.stamp.nanosec = now; break;
      case REAR: last_rear_msg = *msg; last_rear_msg.header.stamp.nanosec = now;    break;
    }
  }

  void publish_topic(){
    pub_ams->publish(last_ams_msg);
    pub_bamo->publish(last_bamo_msg);
    pub_front->publish(last_front_msg);
    pub_rear->publish(last_rear_msg);
  }

  void debugger(){
    // int64_t now = this->get_clock()->now().nanoseconds();
    // RCLCPP_INFO(this->get_logger(),"0.TIME: %ld", now);
    /*Time function is valid*/

    RCLCPP_INFO(this->get_logger(),"1.AMS CAN Frame: %0x", last_ams_msg.id);
    RCLCPP_INFO(this->get_logger(),"2.BAMO CAN Frame: %0x", last_bamo_msg.id);
    RCLCPP_INFO(this->get_logger(),"3.FRONT Frame: %0x", last_front_msg.id);
    RCLCPP_INFO(this->get_logger(),"4.REAR Frame: %0x", last_rear_msg.id);
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