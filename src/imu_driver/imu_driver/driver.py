#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from imu_msg.msg import IMUmsg
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
import serial
import numpy as np
import serial
from datetime import datetime

class IMUDriver(Node):
    
    def __init__(self):
        super().__init__('imu_driver')   #EXECUTABLE NAME

        #DECLARING THE PORT AND TAKING INPUT    
        self.declare_parameter('port', '/dev/ttyUSB0')  
        port = self.get_parameter('port').get_parameter_value().string_value
        

        #CREATING A PUBLISHER
        self.publisher_ = self.create_publisher(IMUmsg, '/imu', 10) 
        self.serial_connection = serial.Serial(port, baudrate=115200, timeout=1.0)
        self.get_logger().info(f"Connected to {port}, baudrate: 115200")
        self.serial_connection.write(b'$VNWRG,07,28*27\r\n')

        #CREATING TIMER FOR 40Hz FREQ
        self.create_timer(0.025, self.publish_imu_data)  

    #PUBLISHING DATA
    def publish_imu_data(self):
        line = self.serial_connection.readline().decode("utf-8")

        #TAKING ONLY "VNYMR" DATA
        if line.startswith('$VNYMR'):  
            parsed_data = self.parse_vnymr(line)
            #if parsed_data:
            #    self.publisher_.publish(parsed_data)

    #PARSING DATA 
    def parse_vnymr(self, vnymr_line):
        parts = vnymr_line.split(',')
        if len(parts) < 13:
            return None

        # PARSING ORIENTATION DATA
        roll = float(parts[1]) * np.pi / 180.0   
        pitch = float(parts[2]) * np.pi / 180.0  
        yaw = float(parts[3]) * np.pi / 180.0    
        qw, qx, qy, qz = self.e_to_q(roll, pitch, yaw)

        # PARSING MAG_FIELD, ACCLN AND GYRO DATA
        magx = float(parts[4]) * 1e-4
        magy = float(parts[5]) * 1e-4
        magz = float(parts[6]) * 1e-4

        acclx = float(parts[7])
        accly = float(parts[8])
        acclz = float(parts[9])
        gyrox = float(parts[10])
        gyroy = float(parts[11])
        gyroz = float(parts[12].split('*')[0])  

        #CREATING THE MESSAGE   
        imu_msg = IMUmsg()

        # TAKING TIME AND CONVERSIONS
        system_time = datetime.now().strftime('%H%M%S.%f')
        hours = int(system_time[:2])      
        minutes = int(system_time[2:4])    
        seconds = float(system_time[4:])   

        total_seconds = (hours * 3600) + (minutes * 60) + int(seconds)
        nanoseconds = seconds - int(seconds)
        nanoseconds = int(nanoseconds * 1e9)

        # ASSIGNNING HEADER, TIME(s and ns), FRAME ID
        imu_msg.header.stamp.sec = total_seconds
        imu_msg.header.stamp.nanosec = nanoseconds
        imu_msg.header.frame_id = "IMU1_Frame"

        imu_msg.imu.header.stamp = imu_msg.header.stamp  # Propagate the timestamp
        imu_msg.imu.header.frame_id = imu_msg.header.frame_id  # Propagate the frame_id
        imu_msg.mag_field.header.stamp = imu_msg.header.stamp  # Propagate the timestamp
        imu_msg.mag_field.header.frame_id = imu_msg.header.frame_id  # Propagate the frame_id

        # SENDING DATA TO THE PUBLISHER USING CUSTOM MSG
        imu_msg.imu.orientation.x = qx
        imu_msg.imu.orientation.y = qy
        imu_msg.imu.orientation.z = qz
        imu_msg.imu.orientation.w = qw

        imu_msg.imu.angular_velocity.x = gyrox
        imu_msg.imu.angular_velocity.y = gyroy
        imu_msg.imu.angular_velocity.z = gyroz

        imu_msg.imu.linear_acceleration.x = acclx
        imu_msg.imu.linear_acceleration.y = accly
        imu_msg.imu.linear_acceleration.z = acclz

        # Fill in the Magnetic Field data
        imu_msg.mag_field.magnetic_field.x = magx
        imu_msg.mag_field.magnetic_field.y = magy
        imu_msg.mag_field.magnetic_field.z = magz

        # PRINTING THE DATA ON TERMINAL (TO CROSS CHECK)
        self.get_logger().info(f"Published IMU data - "
                            f"\nOrientation: (w={qw}, x={qx}, y={qy}, z={qz}), "
                            f"\nAngular Velocity: (x={gyrox}, y={gyroy}, z={gyroz}), "
                            f"\nLinear Acceleration: (x={acclx}, y={accly}, z={acclz}), "
                            f"\nMagnetic Field: (x={magx}, y={magy}, z={magz})"
                            f"\nTime (Seconds): {total_seconds},"
                            f"\nTime (Seconds): {nanoseconds}")

        # PUBLISHING THE MESSAGE TO '/imu'
        self.publisher_.publish(imu_msg)

        return imu_msg

    #CONVERSION FROM RADIANS TO QUANTERNIONS
    def e_to_q(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy

        return [q_w, q_x, q_y, q_z]

    
def main(args=None):
    rclpy.init(args=args)
    
    imu_driver = IMUDriver()
    rclpy.spin(imu_driver)
    
    imu_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()