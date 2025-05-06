import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class QubeController(Node):
    def __init__(self):
        super().__init__('qube_controller')
        # PID parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('setpoint', 0.0)
        # Anti-windup and filter parameters
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('integral_limit', 1.0)
        self.declare_parameter('derivative_filter_tau', 0.01)

        # Fetch parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.setpoint = self.get_parameter('setpoint').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.tau = self.get_parameter('derivative_filter_tau').value

        # Internal states
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = self.get_clock().now().nanoseconds * 1e-9
        self._last_derivative = 0.0

        # Publisher and subscriber
        self.pub = self.create_publisher(Float64MultiArray,
                                         '/velocity_controller/commands', 10)
        self.sub = self.create_subscription(JointState,
                                           '/joint_states',
                                           self.joint_state_cb, 10)
        self.get_logger().info('QubeController initialized, waiting for joint_states...')
<xacro:include filename="$(find qube_bringup)/ros2_control/qube_driver.ros2_control.xacro"/>

    def joint_state_cb(self, msg: JointState):
        # Assume motor_joint at index 0
        pos = msg.position[0]
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self._last_time if self._last_time else 0.0
        error = self.setpoint - pos

        # Integral with anti-windup clamp
        self._integral += error * dt
        self._integral = max(min(self._integral, self.integral_limit), -self.integral_limit)

        # Derivative with first-order low-pass filter
        raw_derivative = (error - self._last_error) / dt if dt > 0 else 0.0
        alpha = self.tau / (self.tau + dt) if dt > 0 else 0.0
        derivative = alpha * self._last_derivative + (1 - alpha) * raw_derivative

        # PID output
        u = self.kp * error + self.ki * self._integral + self.kd * derivative
        # Saturate output
        u = max(min(u, self.max_velocity), -self.max_velocity)

        # Publish command
        cmd = Float64MultiArray(data=[u])
        self.pub.publish(cmd)

        # Update states
        self._last_error = error
        self._last_time = now
        self._last_derivative = derivative


def main(args=None):
    rclpy.init(args=args)
    node = QubeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

