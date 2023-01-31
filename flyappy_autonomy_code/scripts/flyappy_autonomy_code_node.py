#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np


class FlyappyRos:
    def __init__(self):
        # Publisher for sending acceleration commands to Flyappy
        self._pub_acc_cmd = rospy.Publisher(
            "/flyappy_acc",
            Vector3,
            queue_size=1
        )

        # Subscribers to topics from Flyappy game
        self._sub_vel = rospy.Subscriber(
            "/flyappy_vel",
            Vector3,
            self.velocity_callback
        )
        self._sub_laser_scan = rospy.Subscriber(
            "/flyappy_laser_scan",
            LaserScan,
            self.laser_scan_callback
        )
        self._sub_game_ended = rospy.Subscriber(
            "/flyappy_game_ended",
            Bool,
            self.game_ended_callback
        )

        self.laser_data = None
        self.current_goal = 0
        self.angles = np.linspace(-45, 45, 9)

    def velocity_callback(self, msg: Vector3) -> None:
        # P gain
        p = 56.0    
    
        vx = 0.9*np.cos(self.current_goal) 
        vy = 0.05*self.current_goal

        ax = p*(vx - msg.x)
        ay = p*(vy - msg.y)

        self._pub_acc_cmd.publish(Vector3(ax,ay,0))


    def laser_scan_callback(self, msg: LaserScan) -> None:
        self.laser_data = msg.ranges

        # rospy.loginfo_throttle(
        #     1,
        #     f"Laser range: {self.laser_data}"
        # )

    
    def autonomous_driving(self): 
        target_angle = 0  

        while not rospy.is_shutdown():
            # print(f"Laserscan from the class: {self.laser_data}")
            try:
                # Laserscan intensity 
                intensity = np.zeros(9)

                for i in range(len(self.laser_data)):
                    if self.laser_data[i] < 1.5:
                        intensity[i] = 1            # Obstacle
                    else:
                        intensity[i] = 0            # Obstacle free
                
                # Index of all the points that are zero
                intensity = np.where(intensity == 0)[0]
                
                # Sum of index passing through
                target_angle = np.sum(list(self.angles[intensity]))
                
                min_distance = np.min(self.laser_data)

                laser_reading = np.array(self.laser_data)
                average_distance = np.mean(list(laser_reading[[3, 4, 5]]))
                # print(f"average_distance: {average_distance}")

                if average_distance > 2.6:
                    self.current_goal = 0

                else:
                    self.current_goal = target_angle
                    if min_distance < 0.5:
                        self.current_goal = 0
                
            except:
                print("An exception occurred")


    def game_ended_callback(self, msg: Bool) -> None:
        if msg.data:
            rospy.loginfo("Crash detected.")
        else:
            rospy.loginfo("End of countdown.")

        

def main() -> None:
    rospy.init_node('flyappy_autonomy_code', anonymous=True)
    flyappy_ros = FlyappyRos()  # noqa: F841
    flyappy_ros.autonomous_driving()
    rospy.spin()


if __name__ == '__main__':
    main()
