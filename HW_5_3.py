#!/usr/bin/env python3
from re import S
import rclpy
import math 
import numpy

from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from unmanned_systems_ros2_pkg import some_python_module
from unmanned_systems_ros2_pkg import PIDTemplate
from unmanned_systems_ros2_pkg import Pathfinders

def get_time_in_secs(some_node:Node) -> float:
    return some_node.get_clock().now().nanoseconds /1E9
    
def euler_from_quaternion(x:float, y:float, z:float, w:float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



class TurtleBotNode(Node):
    def __init__(self, ns=''):
        super().__init__('minimial_turtlebot')
        
        if ns != '':
            self.ns = ns
        else:
            self.ns = ns
                
        #create vel and odom pub and subscribers
        self.vel_publisher = self.create_publisher(
            Twist, self.ns+ "/cmd_vel" ,  10) 
        
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns +"/odom", self.odom_callback, 10)
        
        self.current_position = [None,None]
        self.orientation_quat = [0,0,0,0] #x,y,z,w
        self.orientation_euler = [0,0,0] #roll, pitch, yaw

    def odom_callback(self,msg:Odometry) -> None:
        """subscribe to odometry"""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        roll,pitch,yaw = euler_from_quaternion(qx, qy, qz, qw)
        
        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch 
        self.orientation_euler[2] = yaw

        if self.orientation_euler[2] <0:
            self.orientation_euler[2] += 2*numpy.pi
        else:
            self.orientation_euler[2] = self.orientation_euler[2]

        
        #print("yaw is", numpy.degrees(self.orientation_euler[2]))
        
    def move_turtle(self, linear_vel:float, angular_vel:float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
    def compute_index(min_x:int, max_x:int, min_y:int, max_y:int, 
                  gs:int, x_curr:int, y_curr:int) ->int:
        index = ((x_curr - min_x)/gs) + ((y_curr - min_y)/gs*(max_x+gs-min_x)/gs)
        return index
    
def main()->None:
    Start_x = 1
    Start_y = 1
    Goal_x = 7
    Goal_y = 13
    Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
    8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
    5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
    Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
    8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
    14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
    max_x = 15
    max_y = 15
    min_x = 0
    min_y = 0
    gs = 0.5
    domain_x = numpy.arange(min_x,max_x+gs,gs)
    domain_y = numpy.arange(min_y,max_y + gs,gs)
    domain_gs = numpy.arange(-gs,gs + gs,gs)    
    startime = Pathfinders.AStar(min_x,min_y,max_x,max_y,gs,domain_x,domain_y,domain_gs)
    wp_list = startime.main(Goal_x,Goal_y,Start_x,Start_y,Obstacle_x,Obstacle_y,)
    rclpy.init(args=None)
    print("starting")
    for i in wp_list:
        if wp_list[i].parent_index == -1:
            current_wp = wp_list[i]
    namespace = ''
    rate_val = 5
    turtlebot_node = TurtleBotNode(namespace)
    rate = turtlebot_node.create_rate(rate_val)
    
    des_x_position = 7.0
    cmd_vel = 1.5
    ang_vel = 0.0
    
    stop_vel = 0.0
    
    time_duration = 12
    time_now = get_time_in_secs(turtlebot_node)
    print("time now is", time_now)
    
    kp_angular = 3.0
    ki_angular = 0.1
    kd_angular = 0.0
    dt_angular = 1/20
    pid_angular = PIDTemplate.PID(kp_angular,ki_angular,kd_angular,dt_angular)
   #set a desired heading angle
    MAX_ANG_SPEED_RAD = 2.84
    error_tolerance = numpy.deg2rad(1)
    distance_error_tol = .1
    #waypoint list
   
    i = 0
    #runs program of turtlebot_node
    try:
        rclpy.spin_once(turtlebot_node)
        
        while rclpy.ok():
            while current_wp.x != Goal_x or current_wp.y != Goal_y:
                #get current heading
                if current_wp.parent_index == -1 and i == 0:
                    current_wp = current_wp
                    i = i + 1
                dy = current_wp.y - turtlebot_node.current_position[1]
                dx = current_wp.x - turtlebot_node.current_position[0]
                des_heading = numpy.arctan2(dy,dx)
                if des_heading <0:
                    des_heading += 2*numpy.pi
                else:
                    des_heading = des_heading
                #print(turtlebot_node.current_position[0],turtlebot_node.current_position[1])
                #print('desired heading is', numpy.rad2deg(des_heading))
                current_heading_error = pid_angular.compute_error(des_heading,turtlebot_node.orientation_euler[2])
                current_distance_error = numpy.sqrt(dx**2 + dy**2)

                #set current heading to pid
                
                while abs(current_heading_error) >= error_tolerance:
                    angular_gains = pid_angular.get_gains(des_heading,
                    turtlebot_node.orientation_euler[2])
                    current_heading_error = pid_angular.compute_error(des_heading,turtlebot_node.orientation_euler[2])
                    # print('my heading error is',numpy.rad2deg(pid_angular.error[0]))
                    # print('my gains are', angular_gains)
                    if angular_gains >= MAX_ANG_SPEED_RAD:
                            angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                            angular_gains = -MAX_ANG_SPEED_RAD
                    
                    turtlebot_node.move_turtle(0.0,angular_gains)
                    rclpy.spin_once(turtlebot_node)
                    
                    
                    #in pid class compute the error
                    #return the gains
                    time_diff = get_time_in_secs(turtlebot_node) - time_now 
                
            
                    if abs(current_heading_error) < error_tolerance:
                        print('im done')
                        print("yaw is", numpy.degrees(turtlebot_node.orientation_euler[2]))
                        break
                    
                while current_distance_error >= distance_error_tol:
                    
                    #print("yaw is", numpy.degrees(turtlebot_node.orientation_euler[2]))
                    angular_gains = pid_angular.get_gains(des_heading,
                    turtlebot_node.orientation_euler[2])
                    current_heading_error = pid_angular.compute_error(des_heading,turtlebot_node.orientation_euler[2])
                    # print('my heading error is',numpy.rad2deg(pid_angular.error[0]))
                    # print('my gains are', angular_gains)
                    if angular_gains >= MAX_ANG_SPEED_RAD:
                            angular_gains = MAX_ANG_SPEED_RAD
                    elif angular_gains <= -MAX_ANG_SPEED_RAD:
                            angular_gains = -MAX_ANG_SPEED_RAD
                    
                    dx = current_wp.x - turtlebot_node.current_position[0]
                    dy = current_wp.y - turtlebot_node.current_position[1]
                    current_distance_error = numpy.sqrt(dx**2 + dy**2)
                    #print('distance error is',current_distance_error)
                    if abs(current_distance_error) < distance_error_tol:
                        turtlebot_node.move_turtle(0.0,0.0)
                        wp_keys = list(wp_list)
                        for wp in wp_keys:
                            if wp_list[wp].parent_index == TurtleBotNode.compute_index(min_x, max_x, min_y, max_y, 
                  gs, current_wp.x, current_wp.y):
                                current_wp = wp_list[wp]
                        print('you made it')
                        print(current_wp.x,current_wp.y)
                        
                        break
                    turtlebot_node.move_turtle(0.15,angular_gains)
                    rclpy.spin_once(turtlebot_node)
                i = i + 1
                    
                    
            # if turtlebot_node.current_position[0] <= des_x_position:
            #     turtlebot_node.move_turtle(cmd_vel, ang_vel)
            #     print("not there yet", turtlebot_node.current_position[0])    
            
            # elif turtlebot_node.current_position[0] >= des_x_position:
            #     turtlebot_node.move_turtle(stop_vel, 0.0)
            #     turtlebot_node.destroy_node()
    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0,0.0)
       

if __name__ == '__main__':
    """apply imported function"""
    main()
