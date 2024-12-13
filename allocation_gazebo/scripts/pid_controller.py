#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Declare parameters with default values
        self.declare_parameter('P', 0.5)
        self.declare_parameter('I', 0.0)
        self.declare_parameter('D', 0.0)
        self.declare_parameter('I_max', 10.0)
        self.declare_parameter('I_min', 0.0)

        # Get initial values
        self.P = self.get_parameter('P').value
        self.I = self.get_parameter('I').value
        self.D = self.get_parameter('D').value
        self.I_max = self.get_parameter('I_max').value
        self.I_min = self.get_parameter('I_min').value

        self.get_logger().info(f"Initial Parameters - P: {self.P}, I: {self.I}, D: {self.D}, I_max: {self.I_max}, I_min: {self.I_min}")

        # Set up a parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'P':
                self.P = param.value
            elif param.name == 'I':
                self.I = param.value
            elif param.name == 'D':
                self.D = param.value
            elif param.name == 'I_max':
                self.I_max = param.value
            elif param.name == 'I_min':
                self.I_min = param.value
        
        self.get_logger().info(f"Updated Parameters - P: {self.P}, I: {self.I}, D: {self.D}, I_max: {self.I_max}, I_min: {self.I_min}")
        return rclpy.parameter.ParameterEventCallbackResult()

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
