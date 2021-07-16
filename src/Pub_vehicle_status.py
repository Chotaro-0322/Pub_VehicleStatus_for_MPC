#!/usr/bin/env python
from autoware_msgs.msg import VehicleStatus, Gear
from itolab_senior_car_msgs.msg import Servo
import std_msgs.msg
import math
import rospy

class Mpc_subscriber(object):
    def __init__(self):
        print("RUN Vehicle Status")
        self.center_steering = 85
        self.steering = 0
        self.accel = 0
        self.reverse = 0

    def servo_callback(self, data):
        print("steering is", data.steering)
        self.steering = data.steering
        self.accel = data.accel
        self.reverse = data.reverse

    def fake_vehicle(self):
        rospy.init_node("fake_status")

        
        pub = rospy.Publisher("/vehicle_status", VehicleStatus, queue_size=100)
        rospy.Subscriber("/servo_cmd", Servo, self.servo_callback)
        
        r = rospy.Rate(5)
        
        gear_msg = Gear()
        gear_msg.gear = 0
        
        std_header = std_msgs.msg.Header()
        std_header.stamp = rospy.Time.now()
        std_header.frame_id = "base_link"
        
        msg = VehicleStatus()
        
        while not rospy.is_shutdown():
            msg.header = std_header
            msg.tm = ""
            msg.drivemode = 0
            msg.steeringmode = 0
            msg.current_gear = gear_msg
            msg.speed = 1.5 # m/s
            msg.drivepedal = 0
            msg.brakepedal = 0
            msg.angle = (self.steering - self.center_steering) * math.pi /180 # rad
            msg.lamp = 0
            msg.light = 0
        
            pub.publish(msg)
        
if __name__=="__main__":
    Mpc_sub = Mpc_subscriber()
    
    try:
        Mpc_sub.fake_vehicle()

    except rospy.ROSInterruptException: pass
