import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState
from custom_interfaces.msg import FloatArr  # Custom message
# from datetime import datetime

# UNIVERSAL CONSTANT
TIMER_PUBLISH_PERIOD_MS = 200 
''' Mk2 of timestamper
Standardized every Node (Except AMS , as Float32 MultiArray (7 degit precision))
AMS will get Battery State Message
    (with Cell voltage as dynamic array , will includes all 80 cells , partition at ten by car design)
    
Having separate subscriber of about x topic , each topic contain all sensor data for one node

Having ROS2 Parameter in the microROS workspace    
''' 

''' Mk3 of timestamper
Move the host side scipt such as this one back to the ROS2 WS, 
microROS_WS , I'll try firmware workspace ,see if I can work with it

if all goes a success, we can make both workspace compile custome msg into its own build file,
soooo
''' 

# Topic list
TopicAMS = 'AMStopic'
TopicBAMO = 'BAMOtopic'
TopicFront = 'FRONTtopic'
TopicRear = 'REARtopic'

class TimestampWrapper(Node):
    def __init__(self):
        super().__init__('timestamp_wrapper')
        
        # Might need to set different QoS profile based on Node priority in later revs
        # self.sub = self.create_subscription(BatteryState, TopicAMS, self.callback_AMS, 10)
        self.sub = self.create_subscription(Float32MultiArray, TopicBAMO, self.callback_sensor_controller, 10)
        # self.sub = self.create_subscription(Float32MultiArray, TopicFront, self.callback_sensor_controller, 10)
        # self.sub = self.create_subscription(Float32MultiArray, TopicRear, self.callback_sensor_controller, 10)
        
        # self.pub = self.create_publisher(FloatArr, TopicAMS +'_stamped', 10)
        self.pub = self.create_publisher(FloatArr, TopicBAMO +'_stamped', 10)
        # self.pub = self.create_publisher(FloatArr, TopicFront +'_stamped', 10)
        # self.pub = self.create_publisher(FloatArr, TopicRear +'_stamped', 10)
        
        ## This is for timebased callback (Periodic)
        # timer_period = TIMER_PUBLISH_PERIOD_MS/1000  # 0.1 seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
            # Anothername will be added
            # Or method might change to log timestamp locally by SNTP

    # Timer callback boilerplate
    def callback_AMS(self,msg:BatteryState):
        
        now = self.get_clock().now().to_msg()
        out_msg = BatteryState()
        
        # Fill sensor data, Fill the header timestamp
        out_msg.header.stamp = now
        # out_msg.header.frame_id = ""
        
        # Is battery connected to AMS
        out_msg.present = msg.present
        out_msg.voltage = msg.voltage
        out_msg.current = msg.current
        out_msg.capacity = msg.capacity # Ah :: power and other params will be calculated as post process
        
        # Point cell voltage to subscribed msg topic
        out_msg.cell_voltage = msg.cell_voltage
        out_msg.cell_temperature = msg.cell_temperature
        
        # Power supply status
        out_msg.power_supply_technology = msg.power_supply_technology   # LiPo
        out_msg.power_supply_health = msg.power_supply_health           # AMS Fault code , overvoltage ,overtemperature , and such (Decision made at AMS, but can be remade by info here)
        out_msg.power_supply_status = msg.power_supply_status           # AMS status , Full, Low Voltage(Use unknown) , discharge or charge, 
        
        # Calculated SOC
        out_msg.percentage = msg.percentage
        
        # Convert timestamp to default ISO format dateTime
        # datetimefromUnix = datetime.fromtimestamp(out_msg.header.stamp.sec)
            # out_msg.unixtime = now.sec
            # out_msg.isodatetime = datetimefromUnix.isoformat()
        
        self.pub.publish(out_msg)
        self.get_logger().info("%s" % out_msg.header)
        self.get_logger().info("%s" % out_msg.sensordata)
    
    ''' Timercallback for those with same data field may use the same callback , since the function purpose is only adding the timestamp header
        No relevance to FloatArray data field
    '''
    
    # Sensor/Controller Node 1-3 : BAMO
    def callback_sensor_controller(self,msg:Float32MultiArray):
        
        now = self.get_clock().now().to_msg()
        out_msg = FloatArr()
        
        # Fill sensor data, Fill the header unix timestamp
        out_msg.header.stamp = now
        # out_msg.header.frame_id = ""
        out_msg.sensordata.data = msg.data
        out_msg.sensordata.layout.data_offset = 0
        
        # Convert timestamp to default ISO format dateTime
        # datetimefromUnix = datetime.fromtimestamp(out_msg.header.stamp.sec)
            # out_msg.unixtime = now.sec
            # out_msg.isodatetime = datetimefromUnix.isoformat()
        
        self.pub.publish(out_msg)
        self.get_logger().info("%s" % out_msg.header)
        self.get_logger().info("%s" % out_msg.sensordata)
        
        
def main(args=None):
    
    rclpy.init(args=args)
    SBC_ROSnode = TimestampWrapper()
    rclpy.spin(SBC_ROSnode)
    SBC_ROSnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
# ISOformat datetime ,can be calculated later by accessing the header timestamp of each msg (Fetching unix sec time)