import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

import numpy as np
import sophuspy as sp

SMALL_EPS = 1e-10
G = np.array([0.0, 0.0, -9.81], dtype=float) # gravity vector

def vector_to_SO3_exp(w):
    """
    Convert a 3-dimensional vector to a rotation matrix in SO(3) using the exponential map.
    
    Parameters:
        w (array_like): 3-dimensional vector representing the angular velocity.
    
    Returns:
        R (numpy.ndarray): 3x3 rotation matrix in SO(3) corresponding to the exponential map of w.
    """
    w_hat = np.array([[0, -w[2], w[1]],
                       [w[2], 0, -w[0]],
                       [-w[1], w[0], 0]])
    theta = np.linalg.norm(w)
    if theta < 1e-12:  # Small angle approximation to avoid division by zero
        return np.eye(3) + w_hat
    else:
        return np.eye(3) + np.sin(theta) / theta * w_hat + (1 - np.cos(theta)) / (theta**2) * np.dot(w_hat, w_hat)

def SO3_exp(omega: np.ndarray):
    theta = np.linalg.norm(omega)
    half_theta = 0.5 * theta
    real_factor = np.cos(half_theta)
    if (theta < SMALL_EPS):
        theta_sq = theta * theta
        theta_po4 = theta_sq * theta_sq
        imag_factor = 0.5 - 0.0208333 * theta_sq + 0.000260417 * theta_po4
        print("small value")

    else:
        sin_half_theta = np.sin(half_theta)
        imag_factor = sin_half_theta / theta

    quat = [real_factor, imag_factor * omega[0], imag_factor * omega[1], imag_factor * omega[2]]
    #quat = quat / np.linalg.norm(quat)
    return R.from_quat((quat[1], quat[2], quat[3], quat[0]))

class PreIntegrationNode(Node):

    def __init__(self):
        super().__init__('pre_integration_node')
        self.subscription = self.create_subscription(
            Imu,
            'front_axle_IMU',
            self.imu_callback,
            1)
        
        self.odom_pub = self.create_publisher(Odometry, "odometry/preintergration", 1)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1, self.reset_filter)

        self.p = np.zeros(3, dtype=float)
        self.R = R.from_quat((0.0, 0.0, 0.0, 1.0))
        #self.R = sp.SO3(np.eye(3)) 
        self.v = np.zeros(3, dtype=float)
        self.time = None
        self.time_total = 0.0


        self.acc_b_n = np.array([1.1770342573712328e-01 + 1.8313517106908594e-03, 1.2380926115748600e-01 + 1.5636022839884459e-03, 1.4496090821613894e-01 + 1.6701917625422918e-03])
        self.gyro_b_n = np.array([1.2929623537931202e-03 + 4.9326296778593754e-05, 1.0107391763884263e-03 + 2.6595010766998328e-05, 9.6042247695229683e-04 +  1.8047752689387385e-05])
        #print(self.p)
        #print(self.R)


    def reset_filter(self):
        self.p = np.zeros(3, dtype=float)
        self.R = R.from_quat((0.0, 0.0, 0.0, 1.0))
        #self.R = sp.SO3(np.eye(3)) 
        self.v = np.zeros(3, dtype=float)

    def imu_callback(self, msg: Imu):
        
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]) - self.acc_b_n#- 1.2153261722841530e-02 - 1.9697230073396788e-0
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]) - self.gyro_b_n #- 3.0854095513544628e-03 - 4.1516025854811883e-05
        if self.time == None:
            self.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            dt = 0.0
        else: 
            dt = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9) - self.time
            dt2 = dt * dt
            dR = SO3_exp(w * dt)
            self.p +=  0.5 * self.R.as_matrix() @ (acc * dt2) + 0.5 * G * dt2 + self.v * dt
            self.v += self.R.as_matrix() @ (acc * dt) + G * dt # in the world coord
            self.R *= dR
        

    

        self.time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 # reset time
        #print(self.R)
        self.time_total += dt
        print(f'Total time taken on the trajectry {self.time_total}')
        self.publish_position()

    def publish_position(self):
        msg = Odometry()
        #r = R.from_matrix(self.R.matrix())
        q = self.R.as_quat()

        if q[3] < 0:
            q *= -1
        q = q / np.linalg.norm(q)    
        self.R = R.from_quat(q)
        
        msg.header.frame_id = "imu_odom"
        msg.child_frame_id = "imu"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.p[0]
        msg.pose.pose.position.y = self.p[1]
        msg.pose.pose.position.z = self.p[2]

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.twist.twist.linear.x = self.v[0]
        msg.twist.twist.linear.y = self.v[1]
        msg.twist.twist.linear.z = self.v[2]

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    pre_integration_node = PreIntegrationNode()

    rclpy.spin(pre_integration_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pre_integration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()