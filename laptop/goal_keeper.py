#!/usr/bin/env python3
"""Penjaga tujuan Nav2.

Menyimpan goal dari RViz (/goal_pose). Saat ada kontrol manual, goal Nav2
dibatalkan. Setelah RESUME_DELAY detik tanpa kontrol manual, goal yang sama
dikirim ulang. Goal yang berhasil atau dibatalkan dari RViz dilupakan.
"""
import copy

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

RESUME_DELAY = 3.0    # detik tanpa kontrol manual sebelum kembali ke tujuan
MANUAL_EPS = 0.001    # ambang perintah manual dianggap aktif
START_MARGIN = 0.3    # detik, toleransi waktu penerimaan goal

IDLE, NAVIGATING, PAUSED = 'IDLE', 'NAVIGATING', 'PAUSED'


class GoalKeeper(Node):
    def __init__(self):
        super().__init__('goal_keeper')
        self.state = IDLE
        self.goal = None
        self.nav_start_ns = 0
        self.last_manual = None

        self.create_subscription(PoseStamped, 'goal_pose', self.cb_goal, 10)
        self.create_subscription(Twist, 'cmd_vel_manual', self.cb_manual, 10)
        self.create_subscription(GoalStatusArray,
                                 'navigate_to_pose/_action/status',
                                 self.cb_status, 10)
        self.cancel_cli = self.create_client(
            CancelGoal, 'navigate_to_pose/_action/cancel_goal')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'Goal keeper siap, kembali ke tujuan setelah {RESUME_DELAY:.1f} s tanpa kontrol manual')

    def mark_started(self):
        self.nav_start_ns = (self.get_clock().now().nanoseconds
                             - int(START_MARGIN * 1e9))

    def finish(self):
        self.state = IDLE
        self.goal = None

    def cb_goal(self, msg):
        self.goal = copy.deepcopy(msg)
        self.mark_started()
        self.state = NAVIGATING
        p = msg.pose.position
        self.get_logger().info(f'Tujuan disimpan: x={p.x:.2f} y={p.y:.2f}')

    def cb_manual(self, msg):
        if (abs(msg.linear.x) < MANUAL_EPS
                and abs(msg.angular.z) < MANUAL_EPS):
            return
        self.last_manual = self.get_clock().now()
        if self.state == NAVIGATING:
            self.pause()

    def pause(self):
        self.state = PAUSED
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())
            self.get_logger().info(
                'Kontrol manual aktif: goal Nav2 dibatalkan, tujuan tetap disimpan')
        else:
            self.get_logger().warn(
                'Layanan cancel Nav2 belum siap, goal lama mungkin masih berjalan')

    def cb_status(self, msg):
        if self.state != NAVIGATING or not msg.status_list:
            return
        latest = max(msg.status_list,
                     key=lambda s: (s.goal_info.stamp.sec,
                                    s.goal_info.stamp.nanosec))
        stamp_ns = (latest.goal_info.stamp.sec * 10**9
                    + latest.goal_info.stamp.nanosec)
        if stamp_ns < self.nav_start_ns:
            return  # status milik goal lama
        if latest.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Tujuan tercapai, ingatan tujuan dihapus')
            self.finish()
        elif latest.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 menggugurkan goal, ingatan tujuan dihapus')
            self.finish()
        elif latest.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal dibatalkan dari RViz, ingatan tujuan dihapus')
            self.finish()

    def tick(self):
        if self.state != PAUSED or self.goal is None or self.last_manual is None:
            return
        idle = (self.get_clock().now() - self.last_manual).nanoseconds / 1e9
        if idle < RESUME_DELAY:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Server navigate_to_pose belum siap',
                                   throttle_duration_sec=2.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(self.goal)
        goal.pose.header.stamp = Time().to_msg()
        self.mark_started()
        self.state = NAVIGATING
        p = goal.pose.pose.position
        self.get_logger().info(f'Kembali ke tujuan: x={p.x:.2f} y={p.y:.2f}')
        self.nav_client.send_goal_async(goal).add_done_callback(self.cb_sent)

    def cb_sent(self, fut):
        try:
            handle = fut.result()
        except Exception as e:
            self.get_logger().error(f'Gagal mengirim ulang goal: {e}')
            self.finish()
            return
        if not handle.accepted:
            self.get_logger().warn('Goal ditolak Nav2, ingatan tujuan dihapus')
            self.finish()


def main():
    rclpy.init()
    node = GoalKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
