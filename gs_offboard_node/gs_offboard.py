
import rclpy
import threading
from rclpy.node import Node
import math
import time
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition,TimesyncStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class GSOffboardControl(Node):

    def __init__(self):
        super().__init__('gs_offboard_control')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile)

        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',qos_profile)

        self.create_subscription(VehicleLocalPosition,"/fmu/out/vehicle_local_position",self.curr_pose,qos_profile)
        #self.attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint,'/fmu/in/trajectory_setpoint',qos_profile)

        self.create_subscription(TimesyncStatus,"/fmu/out/timesync_status",self.time_update,10)
        self.create_subscription(PoseStamped,"/target_pose_pose",self.target_point_listener,qos_profile)
        #self.create_subscription(PoseStamped,"/new_target",self.target_point_listener,qos_profile)
        self.create_subscription(Bool,"/boundry_check",self.trigger_command,qos_profile)
        self.create_subscription(Bool,"/connection_fail_status",self.connection_failed,qos_profile)
        self.create_subscription(String,"/drone7/command",self.gs_command,qos_profile)
        self.create_subscription(String,"/drone7/mode",self.gs_mode,qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',qos_profile)
        
        # Timer at 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.flight_status = self.create_timer(0.02, self.fly_status)

        self.get_logger().info("Offboard control node started")
        self.time = self.get_clock().now().nanoseconds // 1000
        #self.time = self.timestamp
        #self.time_at_arm= 0
        
        # counters
        self.offboard_setpoint_counter = 0
        self.path_counter_circle=0
        self.path_counter_straight=0

        self.declare_parameter("target_system",1)
        self.declare_parameter("target_component",1)
        self.declare_parameter("source_system",1)
        self.declare_parameter("source_component",1)

        self.target_system=self.get_parameter("target_system").value
        self.target_component=self.get_parameter("target_component").value
        self.source_system=self.get_parameter("source_system").value
        self.source_component=self.get_parameter("source_component").value



        self.x =0.0
        self.y =0.0
        self.z =0.0
        self.timestamp=0
        self.key=''
        self.target_set_point_x=0.0
        self.target_set_point_y=0.0
        self.target_set_point_z=-0.5
        self.drone_command=''

        #constants
        self.altitude=-0.5
        self.radius=0.5
        self.distance=0.5

        # safty flags
        self.offboard_mode=False
        self.arm=False
        self.land_request=False
        self.ready2hover=True
        self.disarm=False
        self.path1=False
        self.path2=False
        self.target_point_follower=False
        self.hold=False
        self.take_off=False
        self.emergency_landing_request = False
        self.mocap_connection_failed = False
        self.position_mode=False
        self.manual_mode=False

        threading.Thread(target=self.keyboard_listener,daemon=True).start()

    def keyboard_listener(self):
        print('keyboard listner started')
        while rclpy.ok():
            self.key=input()
            print(f'command recieved = {self.key}')

    def gs_command(self,msg):
        print(f"Subscriber succes:{msg}")
        self.drone_command=msg.data

    def gs_mode(self,msg):
        print(f"Subscriber succes:{msg}")
        self.drone_command=msg.data


    def time_update(self,msg):
        self.timestamp=msg.timestamp

    def connection_failed(self,msg):
        self.mocap_connection_failed=msg.data

    def trigger_command(self, msg):
        self.emergency_landing_request=msg.data
        if self.emergency_landing_request:
            self.get_logger().info("Drone is out of bound")
        else:
            #self.get_logger().info("Drone is in bound")
            pass

    def fly_status(self):
        if self.emergency_landing_request or self.mocap_connection_failed:
            time.sleep(1.0)
            if self.emergency_landing_request or self.mocap_connection_failed:
                self.land_request=True
                self.ready2hover=False
                self.disarm=False
                self.path1=False
                self.path2=False
                self.target_point_follower=False
                self.hold=False
                print(f'emergency landing requested out of bound:{self.emergency_landing_request}, connection lost:{self.mocap_connection_failed}')
            else:
                pass
        else:
            if self.drone_command=='offboard':
                self.offboard_mode=True
                self.land_request=False
                self.get_logger().info("GS sent offboard command")
            elif self.drone_command=='position':
                self.position_mode=True
                self.land_request=False
                self.get_logger().info("GS sent position command")
            elif self.drone_command=='manual':
                self.manual_mode=True
                self.land_request=False
                self.get_logger().info("GS sent manual command")
            elif self.drone_command=='arm':
                self.arm=True
                self.land_request=False
                self.get_logger().info("GS sent arm command")
            elif self.drone_command=='fly':
                self.take_off=True
                self.land_request=False
                self.ready2hover=False
                self.disarm=False
                self.path1=False
                self.path2=False
                self.target_point_follower=False
                self.hold=False
                self.drone_command=''
                self.get_logger().info("GS sent fly command")
            elif self.key=='l'or self.drone_command=='land':
                self.land_request=True
                self.take_off=False
                self.ready2hover=False
                self.disarm=False
                self.path1=False
                self.path2=False
                self.target_point_follower=False
                self.hold=False
                print('GS landing requested')
            elif self.key=='d':
                self.disarm=True
                self.take_off=False
                self.land_request=False
                self.ready2hover=False
                self.path1=False
                self.path2=False
                self.hold=False
                self.target_point_follower=False
                            
            elif self.key=='h' or self.drone_command=='hover':
                self.hold=True
                self.land_request=False
                self.take_off=False
                self.ready2hover=False
                self.disarm=False
                self.path1=False
                self.path2=False
                self.target_point_follower=False
                self.x =self.curr_x
                self.y =self.curr_y
                self.z =self.curr_z
                self.key=''
            elif self.key=='1' or self.drone_command=='straight':
                print("straigh path trigger")
                self.land_request=False
                self.take_off=False
                self.ready2hover=False
                self.disarm=False
                self.path1=True
                self.path2=False
                self.hold=False
                self.target_point_follower=False
                self.key=''
                self.drone_command=''
            elif self.key=='2':
                print("circular path trigger")
                self.land_request=False
                self.take_off=False
                self.ready2hover=False
                self.disarm=False
                self.path2=True
                self.path1=False
                self.hold=False
                self.target_point_follower=False
                self.key=''
            elif self.key=='3':
                self.land_request=False
                self.ready2hover=False
                self.take_off=False
                self.disarm=False
                self.path2=False
                self.path1=False
                self.hold=False
                self.target_point_follower=True
                self.key=''

    def target_point_listener(self,msg):
        self.target_set_point_x=msg.pose.position.x
        self.target_set_point_y=msg.pose.position.y
        self.target_set_point_z=msg.pose.position.z

    def curr_pose(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z


    def timer_callback(self):

        # Publish offboard heartbeat
        self.publish_offboard_control_mode()
        
        self.offboard_setpoint_counter += 1

        # Wait a bit before arming and switching mode
        if self.offboard_mode:
            self.get_logger().info("Switching to OFFBOARD mode")

            # Set OFFBOARD mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1.0,param2=6.0)
            self.get_logger().info("OFFBOARD mode set")
            self.offboard_mode=False
            self.drone_command=''
        elif self.position_mode:
            self.get_logger().info("Switching to POSITION mode")

            # Set OFFBOARD mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1.0,param2=3.0)
            self.get_logger().info("POSITION mode set")
            self.position_mode=False
            self.drone_command=''
        elif self.manual_mode:
            self.get_logger().info("Switching to MANUAL mode")

            # Set OFFBOARD mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1.0,param2=1.0)
            self.get_logger().info("MANUAL mode set")
            self.manual_mode=False
            self.drone_command=''
        elif self.arm:
            self.get_logger().info("Arming")

            # Arm
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=1.0)
            self.arm=False
            self.drone_command=''
            time.sleep(1.5)
            self.get_logger().info("Ready to takeoff")
        elif self.take_off:
            self.publish_trajectory_setpoint()
        elif self.path1:
            self.straight_path_setpoint(self.altitude)
            self.path_counter_straight+=1
        elif self.path2:
            self.circular_path_setpoint(self.altitude)
            self.path_counter_circle+=1
        elif self.land_request:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            #self.drone_command=''
        elif self.disarm:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=0.0)
        elif self.target_point_follower:
            self.target_point()
        elif self.hold:
            self.hover(self.x,self.y,self.z)

        # if self.ready2hover:
        #     if self.offboard_setpoint_counter==150:
        #         # self.get_logger().info("Switching to OFFBOARD mode")
        #         # print(f'counter at {self.offboard_setpoint_counter}')

        #         # # Set OFFBOARD mode
        #         # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1.0,param2=6.0)

        #         print(f'command send at  counter {self.offboard_setpoint_counter}')
        #         self.get_logger().info("Arming")

        #         # Arm
        #         self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=1.0)
            
        #     elif self.offboard_setpoint_counter>300:
        #         # Publish desired position
        #         self.publish_trajectory_setpoint()

        # elif self.path1:
        #     self.straight_path_setpoint(self.altitude)
        #     self.path_counter_straight+=1

        
        # elif self.path2:
        #     self.circular_path_setpoint(self.altitude)
        #     self.path_counter_circle+=1

        # elif self.land_request:
        #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        # elif self.disarm:
        #     self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=0.0)
        # elif self.target_point_follower:
        #     self.target_point()
        # elif self.hold:
        #     self.hover(self.x,self.y,self.z)
    def hover(self,x,y,z):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.position = [x,y,z]
        msg.yaw=0.0
        self.trajectory_setpoint_pub.publish(msg)

    def straight_path_setpoint(self,z):
        # angle increment by 0.02 rad
        angle = self.path_counter_straight*0.02
        x = self.distance*math.sin(angle)
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.position = [x,0.0,z]
        msg.yaw=0.0
        self.trajectory_setpoint_pub.publish(msg)

    def target_point(self):
        
        x = self.target_set_point_x
        y = self.target_set_point_y
        z = self.target_set_point_z


        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.position = [x,y,z]
        
        self.trajectory_setpoint_pub.publish(msg)

    def circular_path_setpoint(self, z):
        # angle increment by 0.02 rad
        angle= self.path_counter_circle*0.02
        x=self.radius*math.cos(angle)
        y=self.radius*math.sin(angle)

        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.position=[x,y,z]
        #vx = -R*math.sin(angle)
        #vy = R*math.cos(angle)
        #yaw = math.atan2(vy,vx)
        msg.yaw=0.0
        self.trajectory_setpoint_pub.publish(msg)

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp
        #msg.timestamp =self.get_clock().now().nanoseconds // 1000

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp
        #msg.timestamp =self.get_clock().now().nanoseconds // 1000

        # Position setpoint
        msg.position = [0.0, 0.0, -0.5]

        # Yaw angle
        msg.yaw = 0.0

        self.trajectory_setpoint_pub.publish(msg)


    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0

    ):

        msg = VehicleCommand()

        msg.timestamp =self.get_clock().now().nanoseconds // 1000

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        msg.target_system = self.target_system
        msg.target_component = self.target_component

        msg.source_system = self.source_system
        msg.source_component = self.source_component

        msg.from_external = True

        self.vehicle_command_pub.publish(msg)


def main():

    rclpy.init()

    node = GSOffboardControl()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
