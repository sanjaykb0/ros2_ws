import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, RotaryEncoder

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')

        # Set up the subscriber for cmd_vel (to receive velocity commands)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Define motor connections (replace GPIO pins with your actual wiring)
        self.motor_a = Motor(forward=17, backward=18)  # Motor A control pins
        self.motor_b = Motor(forward=22, backward=23)  # Motor B control pins

        # Define encoder connections (replace GPIO pins with your actual wiring)
        self.encoder_a = RotaryEncoder(24, 25)  # Encoder A pins
        self.encoder_b = RotaryEncoder(26, 27)  # Encoder B pins (if needed)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from the Twist message
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Control the motors based on the received velocities
        if linear_velocity > 0:
            self.move_forward(linear_velocity)
        elif linear_velocity < 0:
            self.move_backward(-linear_velocity)
        else:
            self.stop_motors()

        if angular_velocity != 0:
            self.turn(angular_velocity)

    def move_forward(self, speed):
        """Move the car forward at the specified speed."""
        self.motor_a.forward(speed)
        self.motor_b.forward(speed)
        self.get_logger().info(f'Moving forward at speed {speed}')

    def move_backward(self, speed):
        """Move the car backward at the specified speed."""
        self.motor_a.backward(speed)
        self.motor_b.backward(speed)
        self.get_logger().info(f'Moving backward at speed {speed}')

    def stop_motors(self):
        """Stop the car by stopping both motors."""
        self.motor_a.stop()
        self.motor_b.stop()
        self.get_logger().info('Stopping the car.')

    def turn(self, angular_velocity):
        """Turn the car based on the angular velocity."""
        if angular_velocity > 0:
            self.motor_a.forward(0.5)
            self.motor_b.backward(0.5)
            self.get_logger().info(f'Turning left with angular velocity {angular_velocity}')
        else:
            self.motor_a.backward(0.5)
            self.motor_b.forward(0.5)
            self.get_logger().info(f'Turning right with angular velocity {angular_velocity}')

def main(args=None):
    rclpy.init(args=args)

    # Create the motor control node
    motor_control = MotorControl()

    # Spin the node to keep the program alive and responsive to callbacks
    rclpy.spin(motor_control)

    # Cleanup and shutdown
    motor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

     

   
