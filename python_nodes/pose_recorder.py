import csv

from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from std_srvs.srv import Trigger


class PoseRecorder(Node):

    def __init__(self, exp_num):
        super().__init__("pose_logger")

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        self.create_service(Trigger, '~/record_pose', self.record_pose_cb)

        self.save_name = "./calibration_poses.csv"
        self.current_pose = PoseStamped()
        self.n_poses = 0

        self.pose_file = open(self.save_name, 'w', newline='')
        self.pose_writer = csv.writer(self.pose_file)
        self.pose_writer.writerow(
            ['n', 'pos_x', 'pos_y', 'pos_z',
             'quat_x', 'quat_y', 'quat_z', 'quat_w'])

        self.get_logger().info("READY")


    def pose_callback(self, msg):
        self.current_pose = msg

    def record_pose_cb(self, _req, res):
        self.n_poses += 1
        self.write_pose(self.current_pose)
        return res

    def write_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.pose_writer.writerow([
            self.n_poses,
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w
        ])

    def destroy_node(self):
        closed_any = False
        for name in ('pose_file'):
            f = getattr(self, name, None)
            if f is not None and not f.closed:
                f.close()
                closed_any = True
        if closed_any:
            print("Recording done.")

        super().destroy_node()