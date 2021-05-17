#!/usr/bin/env python

# Import packages
import rospy
from rpi_car_line_follower.msg import IrSensor
import RPi.GPIO as GPIO

#
def ir_sensor_node():
    # Create publisher (topic, message type, queue/buffer size)
    pub = rospy.Publisher('infra_readings', IrSensor, queue_size=10)

    # Initialize node
    rospy.init_node('ir_sensor_node', anonymous=True)

    # Set the loop rate
    rate = rospy.Rate(30) #30 hz

    #--- 'Keep publishing until Ctrl-C is pressed'
    while not rospy.is_shutdown():
        ir_sensor = IrSensor()
        ir_sensor.ir1 = GPIO.input(11)
        ir_sensor.ir2 = GPIO.input(13)
        ir_sensor.ir3 = GPIO.input(15)
        ir_sensor.ir4 = GPIO.input(16)
        ir_sensor.ir5 = GPIO.input(18)
        # rospy.loginfo("Publishing ir sensors:")
        # rospy.loginfo(ir_sensor)
        pub.publish(ir_sensor)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Setup GPIO mode and warnings
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Setup GPIO pins for 'infrared sensor'
        GPIO.setup(11, GPIO.IN)
        GPIO.setup(13, GPIO.IN)
        GPIO.setup(15, GPIO.IN)
        GPIO.setup(16, GPIO.IN)
        GPIO.setup(18, GPIO.IN)

        # 'Run ir sensor node'
        ir_sensor_node()

    except rospy.ROSInterruptException:
        pass

    finally:
        GPIO.cleanup()

        # COMMENTS
