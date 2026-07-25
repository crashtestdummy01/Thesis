#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped

class CartesianTrajectoryStreamer(Node):
    def __init__(self):
        super().__init__('cartesian_trajectory_streamer')
        
        # Publisher for the global target pose topic
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Load your precalculated Cartesian trajectory array (Shape: N x 6)
        trajectory_path = 'long_trajectory.npy' 
        self.trajectory = np.load(trajectory_path)
        
        self.trajectory[:, 2] = 0.1
        self.trajectory[:, 0] += 0.2
        self.trajectory[:, 1] -= 0.2
        self.trajectory *= 1.3 
        
        self.get_logger().info(f"Loaded Cartesian trajectory with {len(self.trajectory)} timesteps.")
        
        self.current_step = 0
        
        # Timer tracking your time step frequency (0.02s = 50 Hz)
        self.timer_period = 0.02  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_step >= len(self.trajectory):
            self.get_logger().info("Cartesian trajectory finished! Initiating clean node shutdown.")
            
            self.timer.destroy()
            
            raise SystemExit
            return

        # Extract the state vector at time step t (first 3 elements are X, Y, Z)
        state_vector = self.trajectory[self.current_step]
        x_target = float(state_vector[0])
        y_target = float(state_vector[1])
        z_target = float(state_vector[2])
        
        # Construct the PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fr3_link0'  # Robot base frame reference
        
        # Assign position from your .npy array
        msg.pose.position.x = x_target
        msg.pose.position.y = y_target
        msg.pose.position.z = z_target
        
        # Force a constant orientation pointing straight down
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        
        # Publish the target command to the spring loop
        self.publisher_.publish(msg)
        
        if self.current_step % 50 == 0:
            self.get_logger().info(
                f"Step {self.current_step}/{len(self.trajectory)} -> XYZ: [{x_target:.3f}, {y_target:.3f}, {z_target:.3f}]"
            )
            
        self.current_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = CartesianTrajectoryStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
