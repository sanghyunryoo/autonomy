#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from core.msg import CommandFilter


class CommandFilterKeyboardPublisher(Node):
    def __init__(self):
        super().__init__('command_filter_keyboard_publisher')

        self.declare_parameter('topic_name', '/command_filter')
        topic_name = self.get_parameter('topic_name').value

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(CommandFilter, topic_name, qos)

        self.msg = CommandFilter()
        self.msg.allow_linear_vel_forward_x = False
        self.msg.allow_linear_vel_backward_x = False
        self.msg.allow_linear_vel_forward_y = False
        self.msg.allow_linear_vel_backward_y = False

        self.publish_state()

        self.get_logger().info(f'Publishing CommandFilter to: {topic_name}')
        self.get_logger().info('Keys: w=forward_x, s=backward_x, a=forward_y, d=backward_y, q/ESC=quit')

    def toggle_key(self, key: str):
        if key == 'w':
            self.msg.allow_linear_vel_forward_x = not self.msg.allow_linear_vel_forward_x

        elif key == 's':
            self.msg.allow_linear_vel_backward_x = not self.msg.allow_linear_vel_backward_x

        elif key == 'a':
            self.msg.allow_linear_vel_forward_y = not self.msg.allow_linear_vel_forward_y

        elif key == 'd':
            self.msg.allow_linear_vel_backward_y = not self.msg.allow_linear_vel_backward_y

        else:
            return

        self.publish_state()

    def publish_state(self):
        self.publisher.publish(self.msg)

        self.get_logger().info(
            'CommandFilter | '
            f'forward_x={self.msg.allow_linear_vel_forward_x}, '
            f'backward_x={self.msg.allow_linear_vel_backward_x}, '
            f'forward_y={self.msg.allow_linear_vel_forward_y}, '
            f'backward_y={self.msg.allow_linear_vel_backward_y}'
        )


def get_key(settings):
    tty.setraw(sys.stdin.fileno())

    key = ''
    if select.select([sys.stdin], [], [], 0.1)[0]:
        key = sys.stdin.read(1)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)

    node = CommandFilterKeyboardPublisher()
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key in ['q', '\x1b']:
                break

            node.toggle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()