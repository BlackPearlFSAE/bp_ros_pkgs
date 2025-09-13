#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <memory>
#include <string>

/*
Mk2 of timestamper
  Standardized every Node (Except AMS , as Float32 MultiArray (7 degit precision))
  AMS will get Battery State Message
      (with Cell voltage as dynamic array , will includes all 80 cells , partition at ten by car design)
  Having separate subscriber of about x topic , each topic contain all sensor data for one node
  Having ROS2 Parameter in the microROS workspace    
Mk3 of timestamper
  Move the host side scipt such as this one back to the ROS2 WS, 
  microROS_WS , I'll try firmware workspace ,see if I can work with it
  if all goes a success, we can make both workspace compile custome msg into its own build file,
  soooo
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
  std::string TopicAMS = "AMS";
  std::string TopicBAMO = "BAMO";
  std::string TopicFront = "Front";
  std::string TopicRear = "Rear";

  enum TopicType { AMS, BAMO, FRONT, REAR };
  rclcpp::TimerBase::SharedPtr timer_;
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
    timer_ = this->create_wall_timer(TIMER_PUBLISH_PERIOD_MS, std::bind(&TimestamperWrapper::publish_topic, this));
  
  };

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