#!/usr/bin/python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import logging

# Set up basic configuration for logging
logging.basicConfig(level=logging.DEBUG)

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.serial_port = '/dev/ttyACM0'  # Adjust the port based on your Arduino connection
        self.serial_baudrate = 115200
        self.serial_connection = serial.Serial(self.serial_port, self.serial_baudrate)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        #self.wspeed_publisher = self.create_publisher(Float64MultiArray, '/wspeed', 10)
        #self.wangle_publisher = self.create_publisher(Float64MultiArray, '/wangle', 10)

        # Wheel geometry constants
        self.LFy = 0.27428
        self.RFy = 0.27428
        self.RBy = 0.133
        self.LBy = 0.133
        self.LFx = 0.184
        self.RFx = 0.184
        self.RBx = 0.133
        self.LBx = 0.133
        self.Mx = 0.254
    
    def twist_callback(self,msg):
        command = self.convert_twist_to_command(msg)

    def convert_twist_to_command(self,twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        if (angular_velocity !=0):
            target_radius,direction=self.get_target_radius
            formatted_radius = f"{target_radius:04d}"
            output_command = "R" + formatted_radius + direction+'\r'
            self.send_command_to_arduino(output_command)
        if (linear_velocity >0):
            self.send_command_to_arduino("F"+'\r')
        elif (linear_velocity<0):
            self.send_command_to_arduino("B"+'\r')
        else:
            self.send_command_to_arduino("S"+'\r')

    def send_command_to_arduino(self, command):
        print(command)
        previousTime=time.time()
        self.serial_connection.write(command.encode())
        self.serial_connection.flush()
        
        while True:
            line = self.serial_connection.readline().decode().strip()
            if line != "":
                currentTime=time.time()
                print(line)
                print('time difference was: ', str(currentTime-previousTime))
                break           
    """
    def twist_callback(self, msg):
        logging.debug("Received Twist message")
        try:
            target_radius, direction = self.get_target_radius(msg)
            angleLF = self.get_target_angle_LF(target_radius, direction)
            angleRF = self.get_target_angle_RF(target_radius, direction)
            angleLB = self.get_target_angle_LB(target_radius, direction)
            angleRB = self.get_target_angle_RB(target_radius, direction)
            if target_radius != 0:
            	speeds = self.get_steer_speeds(direction, target_radius)
            else:
            	speeds=2.6,2.6,2.6,2.6,2.6,2.6
            self.publish_wheel_speeds(speeds)
            self.publish_wheel_angles([angleLF, angleRF, angleLB, angleRB])
            logging.debug("Published wheel speeds and angles")
        except Exception as e:
            logging.error(f"Error processing Twist message: {e}")
    
    def publish_wheel_speeds(self, speeds):
        msg = Float64MultiArray()
        msg.data = speeds
        self.wspeed_publisher.publish(msg)
        logging.debug(f"Wheel speeds published: {speeds}")

    def publish_wheel_angles(self, angles):
        msg = Float64MultiArray()
        msg.data = angles
        self.wangle_publisher.publish(msg)
        logging.debug(f"Wheel angles published: {angles}")
    """
    def get_target_radius(self, twist_msg):
        linear_velocity = twist_msg.linear.x
        angular_velocity = twist_msg.angular.z
        direction = "R" if angular_velocity < 0 else "L"
        if angular_velocity != 0 :
        	target_radius = abs(linear_velocity / angular_velocity)*1000 
        else:
        	target_radius=0
        if target_radius > 0 and target_radius<500:
        	target_radius=500
        if target_radius > 0 and target_radius>2500:
            target_radius=2500
        return target_radius, direction
    """
    def get_target_angle_LF(self, target_radius,direction):
        if (direction=='r'):
            LF_radius=target_radius+self.LFx
            angle_rad=math.asin(self.LFy/LF_radius)
            angle=int(angle_rad*(180/math.pi))
            angle=angle*(-1)
        else:
            LF_radius=target_radius-self.LFx
            angle_rad=math.asin(self.LFy/LF_radius)
            angle=int(angle_rad*(180/math.pi))
        return angle

    def get_target_angle_RF(self, target_radius,direction):
        if (direction=='r'):
            RF_radius=target_radius-self.RFx
            angle_rad=math.asin(self.RFy/RF_radius)
            angle=int(angle_rad*(180/math.pi))
            angle=angle*(-1)
        else:
            RF_radius=target_radius+self.RFx
            angle_rad=math.asin(self.RFy/RF_radius)
            angle=int(angle_rad*(180/math.pi))
        return angle

    def get_target_angle_RB(self, target_radius,direction):
        if (direction=='r'):
            RB_radius=target_radius-self.RBx
            angle_rad=math.asin(self.RBy/RB_radius)
            angle=int(angle_rad*(180/math.pi))
        else:
            RB_radius=target_radius+self.RBx
            angle_rad=math.asin(self.RBy/RB_radius)
            angle=int(angle_rad*(180/math.pi))
            angle=angle*(-1)
        return angle

    def get_target_angle_LB(self, target_radius,direction):
        if (direction=='r'):
            LB_radius=target_radius+self.LBx
            angle_rad=math.asin(self.LBy/LB_radius)
            angle=int(angle_rad*(180/math.pi))           
        else:
            LB_radius=target_radius-self.LBx
            angle_rad=math.asin(self.LBy/LB_radius)
            angle=int(angle_rad*(180/math.pi))
            angle=angle*(-1)
        return angle

    def get_steer_speeds(self, direction,target_radius):
        max_speed_rad_s=2.6
        if (target_radius<=0.8):
            max_speed_rad_s=0.8*max_speed_rad_s
        if (direction=='r'):
            radius_LF=target_radius+self.LFx
            radius_LB=target_radius+self.LBx
            radius_LM=target_radius+self.Mx
            radius_RF=target_radius-self.RFx
            radius_RB=target_radius-self.RBx
            radius_RM=target_radius-self.Mx

            speedLM=max_speed_rad_s
            
            speedRF=(radius_RF/radius_LM)*speedLM
            speedLB=(radius_LB/radius_LM)*speedLM
            speedRB=(radius_RB/radius_LM)*speedLM
            speedRM=(radius_RM/radius_LM)*speedLM
            speedLF=(radius_LF/radius_LM)*speedLM
        else:
            radius_LF=target_radius-self.LFx
            radius_LB=target_radius-self.LBx
            radius_LM=target_radius-self.Mx
            radius_RF=target_radius+self.RFx
            radius_RB=target_radius+self.RBx
            radius_RM=target_radius+self.Mx

            speedRM=max_speed_rad_s

            speedRF=(radius_RF/radius_RM)*speedRM
            speedLB=(radius_LB/radius_RM)*speedRM
            speedRB=(radius_RB/radius_RM)*speedRM
            speedLM=(radius_LM/radius_RM)*speedRM
            speedLF=(radius_LF/radius_RM)*speedRM
        
        return speedLF,speedRF,speedLM,speedRM,speedLB,speedRB
    """
def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()