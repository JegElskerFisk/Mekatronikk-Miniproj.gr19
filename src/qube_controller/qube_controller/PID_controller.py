import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class QubeController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        # PID parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 1.0),
                ('ki', 0.0),
                ('kd', 0.1),
                ('setpoint', 3.0),
                ('max_velocity', 5.0),
                ('integral_limit', 1.0),
                ('derivative_filter_tau', 0.01)
            ])
        
        # Initialize time tracking
        self._last_time = None
        self._last_error = 0.0
        self._integral = 0.0
        self._last_derivative = 0.0
        self._got_first_message = False  # New flag
        
        # ROS 2 setup
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            qos_profile=10
        )
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )
        self.get_logger().info('PID Controller ready')
        
    def joint_state_cb(self, msg):
        try:
            # Initialize time handling on first message
            current_time = self.get_clock().now()
            if not self._got_first_message:
                self._last_time = current_time
                self._got_first_message = True
                return
                
            # Calculate time difference
            dt = (current_time - self._last_time).nanoseconds * 1e-9
            if dt <= 0:
                return
                
            # Get current joint state
            motor_index = msg.name.index('motor_joint')
            current_pos = msg.position[motor_index]
            
            # Get parameters
            setpoint = self.get_parameter('setpoint').value
            kp = self.get_parameter('kp').value
            ki = self.get_parameter('ki').value
            kd = self.get_parameter('kd').value
            
            # PID calculations
            error = setpoint - current_pos
            self._integral += error * dt
            self._integral = max(min(self._integral, 
                                  self.get_parameter('integral_limit').value), 
                             -self.get_parameter('integral_limit').value)
            
            # Filtered derivative
            raw_derivative = (error - self._last_error) / dt
            alpha = self.get_parameter('derivative_filter_tau').value / \
                   (self.get_parameter('derivative_filter_tau').value + dt)
            derivative = alpha * self._last_derivative + (1 - alpha) * raw_derivative
            
            # Compute and limit output
            control_output = kp * error + ki * self._integral + kd * derivative
            control_output = max(min(control_output, 
                                   self.get_parameter('max_velocity').value), 
                              -self.get_parameter('max_velocity').value)
            
            # Publish command
            self.pub.publish(Float64MultiArray(data=[control_output]))
            
            # Update states
            self._last_error = error
            self._last_time = current_time
            self._last_derivative = derivative
            
        except ValueError as e:
            self.get_logger().warn(f'Joint error: {str(e)}', throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f'Calculation error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    controller = QubeController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
