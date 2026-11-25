import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from math import pi, isfinite


class LeftWallFollower(Node):

    def __init__(self):
        super().__init__('tb3_wall_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)

        self.target_left = 0.4      # 期望左侧墙距
        self.front_danger = 0.35    # 前方危险距离
        self.v = 0.15               # 直行速度
        self.w = 0.6                # 转弯角速度 

        self.front = 10.0
        self.left = 10.0

        self.path = Path()
        self.path.header.frame_id = 'odom'   
        self.point_gap = 0.05            
        self.last_xy = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('左墙跟随节点已启动，轨迹发布到 /robot_path')

    # 雷达回调
    def scan_callback(self, msg):
        n = len(msg.ranges)
        inc = msg.angle_increment
        a_min = msg.angle_min

        def dist_at(angle_deg):
            # 角度换算成ranges索引，并做越界保护
            i = int((angle_deg * pi / 180.0 - a_min) / inc)
            i = max(0, min(n - 1, i))
            d = msg.ranges[i]
            # 过滤无效测距（inf / nan / 0），无效时按"无遮挡"处理
            return d if isfinite(d) and d > 0.0 else 10.0

        # 前方0°±15°；左侧取60°75°90°
        self.front = min(dist_at(0), dist_at(15), dist_at(-15))
        self.left = min(dist_at(60), dist_at(75), dist_at(90))

    #里程计回调：累积轨迹
    def odom_callback(self, msg):
        p = msg.pose.pose.position
        x, y = p.x, p.y

        # 距离上次记录点超过point_gap才追加，避免轨迹点爆炸
        if (self.last_xy is None or
                (x - self.last_xy[0]) ** 2 + (y - self.last_xy[1]) ** 2 >= self.point_gap ** 2):

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = p.z
            pose.pose.orientation = msg.pose.pose.orientation

            self.path.poses.append(pose)

            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)  
            self.last_xy = (x, y)

    def control_loop(self):
        twist = Twist()
        if self.front < self.front_danger:
            # 前方距离过小:原地右转
            twist.linear.x = 0.0
            twist.angular.z = -self.w
        elif self.left > self.target_left + 0.1:
            # 左侧距离偏大:向左微调靠近墙
            twist.linear.x = self.v
            twist.angular.z = 0.4
        elif self.left < self.target_left - 0.1:
            # 左侧太近:向右微调
            twist.linear.x = self.v
            twist.angular.z = -0.3
        else:
            # 左侧保持设定距离:直线前进
            twist.linear.x = self.v
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = LeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()