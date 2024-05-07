import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Imu

import numpy as np

SMALL_EPS = 1e-10

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.p = np.zeros(3)
        self.R = np.identity(3)
        self.v = np.zeros(3)
        self.time = None
        #print(self.p)
        #print(self.R)

    def SO3_exp(self, omega: np.ndarray):
        theta = np.linalg.norm(omega)
        half_theta = 0.5 * theta
        real_factor = np.cos(half_theta)
        if (theta < SMALL_EPS):
            theta_sq = theta * theta
            theta_po4 = theta_sq * theta_sq
            imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4

        else:
            sin_half_theta = np.sin(half_theta)
            imag_factor = sin_half_theta / theta


        return R.from_quat((real_factor, imag_factor * omega[0], imag_factor * omega[1], imag_factor * omega[2])).as_matrix()

    def imu_callback(self, msg: Imu):
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        dR = self.SO3_exp(w)
        if self.time == None:
            self.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 10^(-9)
        
        dt = msg.header.stamp.sec + msg.header.stamp.nanosec * 10^(-9) - self.time
        dt2 = dt * dt
        self.p += self.v * dt + 0.5 * self.R @ acc * dt2
        self.v += self.R @ acc * dt2
        self.R += self.R @ dR


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()