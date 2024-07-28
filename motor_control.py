
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)  # chân 11
GPIO.setup(18, GPIO.OUT)  # chân 12
GPIO.setup(27, GPIO.IN)  # chân 13
GPIO.output(18, 0)
GPIO.output(17, 0)
a = 0
start_time = 0
GPIO.output(17, 0)
GPIO.output(18, 0)

def callback(data):
    global a
    global start_time
    value = data.data
    print("value="+ str(value), end="\n\n")
    if value == 1 and GPIO.input(27) == GPIO.HIGH: #xanh ngoai, vang trong, di xuong
        start_time = time.time()
        a = 1
        GPIO.output(17, 0)
        GPIO.output(18, 1)
    if (time.time()  - start_time) > 1 and a == 1:
        GPIO.output(17, 0)
        GPIO.output(18, 0)
        a = 0
    if value == 0 and GPIO.input(27) == GPIO.LOW:
        GPIO.output(17, 1)
        GPIO.output(18, 0)
    if GPIO.input(27) == GPIO.HIGH and value != 1:
        GPIO.output(17, 0)
        GPIO.output(18, 0)

def subscriber():
    rospy.Subscriber('motor_control', Int32, callback)

if __name__ == '__main__':
    rospy.init_node("motor_control_node")
    rate = rospy.Rate(20)  # 20Hz
    subscriber()

    while not rospy.is_shutdown():
        rate.sleep()


	
