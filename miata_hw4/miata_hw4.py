import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import csv
import os



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
#RIGHT_FRONT_INDEX = 30 # old
#LEFT_FRONT_INDEX = 0 # old
LEFT_SIDE_INDEX=90
WALL_DISTANCE = 0.25 * LIDAR_AVOID_DISTANCE


# in meters
DIST_TO_TRAVEL = 1

# when within DIST_TO_TRAVEL - DIST_DIFF_TO_SLOW_DOWN, it will go at 10mm/sec
# in meters
DIST_DIFF_TO_SLOW_DOWN = 0.05

# speed in m/s
# 0.075 = 75mm/s
# 0.150 = 150mm/s
# DIST_SPEED = 0.15
DIST_SPEED = 0.2


class WallFollow(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('wall_follow_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # CSV for plotting
        # trial_number = 6
        # starting_pos = "area1"
        # print(os.getcwd())
        # self.csv_file = open(f'webots_ros2_homework1_python/webots_ros2_homework1_python/csv/{starting_pos}/trial{trial_number}.csv', mode='a', newline='')
        # self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        # self.csv_writer.writerow(['X', 'Y'])

        self.front_scans = []
        self.left_scans = []
        self.right_scans = []

        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.following_wall = False

        self.starting_point = None
        self.last_rotation_position = None
        self.distance_since_last_rotation = 0
        self.farthest_distance = 0

    # Reads scan and cleans it
    def listener_callback1(self, msg1):
        # self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(5.0)
            #elif reading == float(0.0):
                #self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)

    
    
    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
 
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # similarly for twist message if you need
 
        if self.pose_saved is None:
            self.pose_saved = position
            self.starting_point = position # set the starting point
            self.last_rotation_position = position
        elif self.last_rotation_position is not None:
            self.distance_since_last_rotation = math.sqrt((posx - self.last_rotation_position.x)**2 + (posy - self.last_rotation_position.y)**2)
            # If distance from our last rotation is greater than or equal to some value (going with 0.2m for now), rotate
            if (self.turtlebot_moving and self.distance_since_last_rotation >= 0.2):
                # Do rotation
                self.get_logger().info('START ROTATION HERE!')
                self.distance_since_last_rotation = 0
                self.last_rotation_position = position

        


        self.pose_saved=position
       
           
        return None


    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
            
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        # front_lidar_left = min(self.scan_cleaned[0:30])
        # front_lidar_right = min(self.scan_cleaned[330:])
        # front_lidar_min = min(front_lidar_left, front_lidar_right)
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])



        self.front_scans.append(front_lidar_min)
        self.left_scans.append(left_lidar_min)
        self.right_scans.append(right_lidar_min)

        if (len(self.front_scans) > 10):
            self.front_scans.pop(0)

        if (len(self.left_scans) > 10):
            self.left_scans.pop(0)

        if (len(self.right_scans) > 10):
            self.right_scans.pop(0)

        # If last 10 scans have same distance of something in front and sides, register as stuck
        front_diff_threshold = 0.01 #0.01 used to be
        front_scans_sum = 0
        for scan in self.front_scans:
            front_scans_sum += scan
        front_scans_sum /= 10

        left_diff_threshold = 0.01 #0.01 used to be
        left_scans_sum = 0
        for scan in self.left_scans:
            left_scans_sum += scan
        left_scans_sum /= 10

        right_diff_threshold = 0.01 #0.01 used to be
        right_scans_sum = 0
        for scan in self.right_scans:
            right_scans_sum += scan
        right_scans_sum /= 10

        # If number of scans is equal to 10, sum of scans minus the first scan is less than threshold, and latest scan isn't infinity, register as stuck
        if (((len(self.front_scans) == 10) and (len(self.left_scans) == 10) and (len(self.right_scans) == 10))):
            if ((abs(front_scans_sum - self.front_scans[0]) < front_diff_threshold) and (abs(left_scans_sum - self.left_scans[0]) < left_diff_threshold) and (abs(right_scans_sum - self.right_scans[0]) < right_diff_threshold)):
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.stall = True
                self.get_logger().info('Stuck on something, stopping')

        if (not self.following_wall):
            self.cmd.linear.x = DIST_SPEED
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Moving forward at start?')
            if (right_lidar_min < 3 * WALL_DISTANCE or front_lidar_min < LIDAR_AVOID_DISTANCE):
                self.following_wall = True
            self.turtlebot_moving = True
        elif self.stall:
            self.cmd.linear.x = -0.1 
            self.cmd.angular.z = 0.4 
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            self.following_wall = False
            self.stall = False
            self.get_logger().info('Trying to get out of stall')
        # Detect potential stall
        elif front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0 
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.stall = True
                self.get_logger().info('Stopping')
                return
        # If object in way, just turn left until we can move forward
        elif front_lidar_min < 0.9 * LIDAR_AVOID_DISTANCE:
            self.cmd.linear.x = 0.07 
            self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Turning')
            self.turtlebot_moving = True
            # Once wall is to right, mark the bot as following the wall
            if right_lidar_min <= 3 * WALL_DISTANCE:
                self.following_wall = True
        # Once the bot is following the wall, adjust left and right movement to ensure we stay close to wall
        elif self.following_wall:
            # Assume going straight forward
            self.cmd.linear.x = 0.4
            self.cmd.angular.z = 0.0
            self.get_logger().info('Going straight alongside wall')
            # Adjust if needed
            if right_lidar_min < WALL_DISTANCE:
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = 0.4
                self.get_logger().info('Turning left away from wall')
            elif right_lidar_min > 3 * WALL_DISTANCE:
                # Sanity check
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = -0.4
                self.get_logger().info('Turning right towards wall')
            else:
                self.get_logger().info('Going straight alongside wall')

            self.publisher_.publish(self.cmd)
            self.get_logger().info('Following wall')
            self.turtlebot_moving = True



        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('Distance of the left obstacle : %f' % left_lidar_min)
        self.get_logger().info('Distance of the right obstacle : %f' % right_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    # random_walk_node = RandomWalk()
    wall_follow_node = WallFollow()

    # pause the program execution, waits for a request to kill the node (ctrl+c)
    # rclpy.spin(random_walk_node)
    rclpy.spin(wall_follow_node)

    # Explicity destroy the node
    # random_walk_node.destroy_node()
    wall_follow_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
