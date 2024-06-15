#!/usr/bin/env python3 

import time
import rclpy
from rclpy.node import Node 
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import Countntil 

class CountUntilServer(Node):
    def __init__(self): 
        super().__init__("count_until_server")
        self.count_until_server=ActionServer(
            self , Countntil , count_until , execute_callback= )
        
        self.get_logger().info("server started ...")

    def execute_callback(self , goal_handle : ServerGoalHandle):
        target_number = goal_handle.goal_number
        period = goal_handle.period 


        self.get_logger().info("Executing the goal")
        counter = 0 
        for i in range (target_number):
            counter+=1 
            self.get_logger().info(str(counter))
            time.sleep(period)


        goal_handle.succeed()


        result=Countntil.result()
        result.reached_number  = counter 
        return result 




def main(args=None): 
    rclpy.init(args=args)
    node=  CountUntilServer()
    rclpy.spin(node)

    rclpy.shutdown()