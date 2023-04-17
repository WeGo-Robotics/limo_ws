#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def main():
    rospy.init_node('lidar_publisher_node', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz publishing rate

    lidar_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    while not rospy.is_shutdown():
        # Assuming that the lidar data is being read from some external source
        ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        intensities = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Create a LaserScan message and fill it with the lidar data
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.1
        scan.time_increment = 0.0
        scan.range_min = 0.0
        scan.range_max = 100.0
        scan.ranges = ranges
        scan.intensities = intensities

        # Publish the LaserScan message
        lidar_pub.publish(scan)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

    #클래스화