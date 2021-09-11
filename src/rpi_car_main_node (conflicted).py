#!/usr/bin/env python3

#--- Import packages
import rospy
from motor import Motor
from rpi_car_line_follower.msg import IrSensor

#--- Subscriber call back function
def infra_readings_callback(ir_sensor_data):

    # Infrared data received from the 5 channel infrared sensor
    ir_1 = ir_sensor_data.ir1
    ir_2 = ir_sensor_data.ir2
    ir_3 = ir_sensor_data.ir3
    ir_4 = ir_sensor_data.ir4
    ir_5 = ir_sensor_data.ir5

    # IR controls
    """    
        ir reading of 0 - black line
        ir reading of 1 - white 'surface'
    """
    if (ir_1 == 1 and ir_2 == 1 and ir_3 == 0 and ir_4 == 1 and ir_5 == 1):
        # Move robot forward
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 1 and ir_2 == 0 and ir_3 == 0 and ir_4 == 1 and ir_5 == 1):
        # Move robot slightly left
        Motor.MotorRun(0, 'forward', 65)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 1 and ir_2 == 0 and ir_3 == 1 and ir_4 == 1 and ir_5 == 1):
        # Move robot further to the left 
        Motor.MotorRun(0, 'forward', 50)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 0 and ir_2 == 0 and ir_3 == 1 and ir_4 == 1 and ir_5 == 1):
        # Move robot even further to the left, slowing down rotation on the left 
        Motor.MotorRun(0, 'forward', 25)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 0 and ir_2 == 1 and ir_3 == 1 and ir_4 == 1 and ir_5 == 1):
        # Move robot left, rotation about left wheel
        Motor.MotorRun(0, 'forward', 10)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 1 and ir_2 == 1 and ir_3 == 0 and ir_4 == 0 and ir_5 == 1):  
        # Move robot slightly right
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 65)
    elif (ir_1 == 1 and ir_2 == 1 and ir_3 == 1 and ir_4 == 0 and ir_5 == 1):
        # Move robot further to the right 
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 50)
    elif (ir_1 == 1 and ir_2 == 1 and ir_3 == 1 and ir_4 == 0 and ir_5 == 0):
        # Move robot even further to the right, slowing down rotation on the right 
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 30)
    elif (ir_1 == 1 and ir_2 == 1 and ir_3 == 1 and ir_4 == 1 and ir_5 == 0):
        # Move robot right, rotation about right wheel
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 15)
    elif (ir_1 == 0 and ir_2 == 0 and ir_3 == 0 and ir_4 == 1 and ir_5 == 1):
        # Additional robot move for figure 8 track
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 10)
    elif (ir_1 == 0 and ir_2 == 0 and ir_3 == 0 and ir_4 == 0 and ir_5 == 1):
        # Additional robot move for figure 8 track
        Motor.MotorRun(0, 'forward', 20)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 1 and ir_2 == 1 and ir_3 == 0 and ir_4 == 0 and ir_5 == 0):
        # Additional robot move for figure 8 track
        Motor.MotorRun(0, 'forward', 70)
        Motor.MotorRun(1, 'forward', 45)
    elif (ir_1 == 1 and ir_2 == 0 and ir_3 == 0 and ir_4 == 0 and ir_5 == 1):
        # Additional robot move for figure 8 track
        Motor.MotorRun(0, 'forward', 45)
        Motor.MotorRun(1, 'forward', 70)
    elif (ir_1 == 0 and ir_2 == 0 and ir_3 == 0 and ir_4 == 0 and ir_5 == 0):
        # Stop Motors
        rospy.loginfo("Stop motors")
        Motor.MotorStop(0)
        Motor.MotorStop(1)
    
    # Change description of different sensor robot movement modes
#
def rpi_car_main():

    # Initialize 'rpi_car_main_node'
    rospy.init_node('rpi_car_main_node', anonymous=True)        

    while not rospy.is_shutdown():
        # Subscribe to infrared_readings topic
        rospy.Subscriber('infra_readings', IrSensor, infra_readings_callback)

        # Keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    # Initialize motor driver
    Motor = Motor()

    while not rospy.is_shutdown():
        try:
            rpi_car_main()
        except rospy.ROSInterruptException:
            pass
    
    # Stop motors after the node is shutdown
    Motor.MotorStop(0)
    Motor.MotorStop(1)
