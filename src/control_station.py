#!/usr/bin/env python3
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import rospy
from curtsies import Input



class Scheduler:
    def __init__(self):
        rospy.init_node("Control_Station", anonymous=False)
        self.takeoffPublisher               = rospy.Publisher("/takeOff", Bool, queue_size=2)
        self.offboardPublisher              = rospy.Publisher("/offboard", Bool, queue_size=2)
        self.landPublisher                  = rospy.Publisher("/landing", Bool, queue_size=2)
        self.positionPublisher              = rospy.Publisher("/target_position", PoseStamped, queue_size=2)

    def takeoff(self):
        rate = rospy.Rate(10)
        takeoff = Bool()
        land = Bool()
        takeoff = True
        land = False
        
        for i in range(0,10):
            self.landPublisher.publish(land)
            rate.sleep()

        for i in range(0,10):
            self.takeoffPublisher.publish(takeoff)
            self.offboardPublisher.publish(takeoff)
            rate.sleep()

        rospy.sleep(10)


    def land(self):
        rate = rospy.Rate(10)
        takeoff = Bool()
        land = Bool()
        takeoff = False
        land = True
        
        for i in range(0,10):
            self.takeoffPublisher.publish(takeoff)
            self.offboardPublisher.publish(takeoff)
            self.landPublisher.publish(land)
            rate.sleep()


    def scan(self):
        rate = rospy.Rate(10)
        position = PoseStamped()

        # AISLE 1
        position.pose.position.x = 0
        position.pose.position.y = -22
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 1
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(45)

        # Traverse to Aisle 2
        position.pose.position.x = -9.5
        position.pose.position.y = -22
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 1
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(15)

        # AISLE 2
        position.pose.position.x = -9.5
        position.pose.position.y = 0
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 1
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(45)

        # Traverse to Aisle 3
        position.pose.position.x = -19
        position.pose.position.y = 0
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 1
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(15)
        
        # AISLE 3
        position.pose.position.x = -19
        position.pose.position.y = -22
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 1
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(45)

        # Traverse to Aisle 3
        position.pose.position.x = -26
        position.pose.position.y = -22
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 0
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(15)

        # Reverse Aisle 3
        position.pose.position.x = -26
        position.pose.position.y = 0
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 0
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(45)

        # Traverse to Aisle 2
        position.pose.position.x = -17
        position.pose.position.y = 0
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 0
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(15)

        # Reverse Aisle 2
        position.pose.position.x = -17
        position.pose.position.y = -22
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 0
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(45)

        # Traverse to Aisle 1
        position.pose.position.x = -8
        position.pose.position.y = -22
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 0
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(5)

        # Reverse Aisle 2
        position.pose.position.x = -8
        position.pose.position.y = 0
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 0
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(45)

        # Traverse to Home
        position.pose.position.x = 0
        position.pose.position.y = 0
        position.pose.position.z = 3
        position.pose.orientation.x = 0
        position.pose.orientation.y = 0
        position.pose.orientation.z = 1
        position.pose.orientation.w = 0
        
        for i in range(0,10):
            self.positionPublisher.publish(position)
            rate.sleep()

        rospy.sleep(15)


def main():
    uav = Scheduler()
    key=0
    key_takeoff         = "'<Ctrl-t>'"
    key_land            = "'<Ctrl-l>'"
    key_scan            = "'<Ctrl-a>'"
    key_break           = "'b'"
    print("Control Station")


    with Input(keynames='curtsies') as input_generator:
        for i in input_generator:
            msg  = [0] * 15
            key     = repr(i)
            print(key)
            if key == key_takeoff or key_land or key_scan or key == key_break:
                if  key==key_takeoff:
                    print("takeoff")
                    uav.takeoff()
                if  key==key_land:
                    print("Landing")
                    uav.land()
                if  key==key_scan:
                    print("Scanning")
                    uav.scan()
                if key==key_break:
                    break
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
