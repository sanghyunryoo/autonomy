#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from core.msg import CommandUser
from core.msg import EventUser


ROS_DOMAIN_ID = "0"
USER_ODOM_TOPIC = "/control_command/user_odom"

CONTROL_DT = 0.1  # 10 Hz

SPEED_STEP = 0.1
MAX_LINEAR_X = 1.0
MAX_ANGULAR_Z = 1.0


EVENT_CODE = {
    "none": 0,
    "forward": 1,
    "backward": 2,
    "rotate_left": 3,
    "rotate_right": 4,
    "stop": 5,
}


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def get_pending_keys(timeout: float = 0.0) -> list[str]:
    keys = []

    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return keys

    keys.append(sys.stdin.read(1))

    while True:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not rlist:
            break
        keys.append(sys.stdin.read(1))

    return keys


def make_command_qos() -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=16,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def try_set_attr(obj, attr_name: str, value) -> bool:
    if not hasattr(obj, attr_name):
        return False

    try:
        setattr(obj, attr_name, value)
    except Exception:
        return False

    return True


def try_set_nested_attr(obj, attr_path: str, value) -> bool:
    parts = attr_path.split(".")
    target = obj

    for part in parts[:-1]:
        if not hasattr(target, part):
            return False
        target = getattr(target, part)

    return try_set_attr(target, parts[-1], value)


def fill_event_user_msg(
    event_msg: EventUser,
    node: Node,
    event_name: str,
) -> None:
    """
    EventUser는 키보드 조작과 무관하게 항상 false 상태로 보낸다.

    core/msg/EventUser 예상 구조:
      bool estop
      bool wake
      bool sleep
      bool rough_drive_toggle
    """

    if hasattr(event_msg, "header"):
        now = node.get_clock().now().to_msg()
        event_msg.header.stamp = now
        event_msg.header.frame_id = "base_link"

    event_msg.estop = False
    event_msg.wake = False
    event_msg.sleep = False
    event_msg.rough_drive_toggle = False

def make_odom_msg(
    node: Node,
    x: float,
    y: float,
    yaw: float,
    linear_x: float,
    angular_z: float,
) -> Odometry:
    now = node.get_clock().now().to_msg()

    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = yaw_to_quaternion(yaw)

    odom.twist.twist.linear.x = linear_x
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0

    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = angular_z

    return odom


def fill_command_user_msg(
    msg: CommandUser,
    odom_msg: Odometry,
    event_msg: EventUser,
) -> None:
    """
    core/msg/CommandUser 구조에 맞춰 직접 채운다.

    실제 구조:
      nav_msgs/Odometry odom
      core/EventUser event
    """

    msg.odom = odom_msg
    msg.event = event_msg


class KeyboardCommandUserTest(Node):
    def __init__(self):
        super().__init__("keyboard_command_user_test")

        self.dt = CONTROL_DT

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.current_event = "none"

        self.command_pub = self.create_publisher(
            CommandUser,
            USER_ODOM_TOPIC,
            make_command_qos(),
        )

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.print_guide()

    def print_guide(self):
        print("")
        print("Keyboard CommandUser test")
        print("-------------------------")
        print(f"ROS_DOMAIN_ID : {os.environ.get('ROS_DOMAIN_ID')}")
        print(f"topic         : {USER_ODOM_TOPIC}")
        print("type          : core/msg/CommandUser")
        print("structure     : nav_msgs/Odometry odom + core/EventUser event")
        print("rate          : 10 Hz")
        print("qos           : keep_last / depth 16 / reliable / volatile")
        print("")
        print("w     : increase forward command by +0.1, max +1.0")
        print("s     : increase backward command by -0.1, max -1.0")
        print("a     : increase left rotation command by +0.1, max +1.0")
        print("d     : increase right rotation command by -0.1, max -1.0")
        print("space : stop and reset command to zero")
        print("q     : quit")
        print("")
        print("Current command is continuously published at 10 Hz.")
        print("")

    def reset_command(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.current_event = "stop"

    def update_pose(self):
        self.x += self.linear_x * math.cos(self.yaw) * self.dt
        self.y += self.linear_x * math.sin(self.yaw) * self.dt
        self.yaw += self.angular_z * self.dt

    def handle_key(self, key: str):
        event_name = None

        if key == "w":
            self.linear_x = clamp(
                self.linear_x + SPEED_STEP,
                -MAX_LINEAR_X,
                MAX_LINEAR_X,
            )
            self.angular_z = 0.0
            event_name = "forward"

        elif key == "s":
            self.linear_x = clamp(
                self.linear_x - SPEED_STEP,
                -MAX_LINEAR_X,
                MAX_LINEAR_X,
            )
            self.angular_z = 0.0
            event_name = "backward"

        elif key == "a":
            self.linear_x = 0.0
            self.angular_z = clamp(
                self.angular_z + SPEED_STEP,
                -MAX_ANGULAR_Z,
                MAX_ANGULAR_Z,
            )
            event_name = "rotate_left"

        elif key == "d":
            self.linear_x = 0.0
            self.angular_z = clamp(
                self.angular_z - SPEED_STEP,
                -MAX_ANGULAR_Z,
                MAX_ANGULAR_Z,
            )
            event_name = "rotate_right"

        elif key == " ":
            self.reset_command()
            event_name = "stop"

        if event_name is not None:
            self.current_event = event_name

            self.get_logger().info(
                f"key={repr(key)}, event={event_name}, "
                f"linear_x={self.linear_x:.2f}, angular_z={self.angular_z:.2f}, "
                f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}"
            )

    def make_command_msg(self) -> CommandUser:
        odom_msg = make_odom_msg(
            node=self,
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_x=self.linear_x,
            angular_z=self.angular_z,
        )

        event_msg = EventUser()
        fill_event_user_msg(
            event_msg=event_msg,
            node=self,
            event_name=self.current_event,
        )

        command_msg = CommandUser()
        fill_command_user_msg(
            msg=command_msg,
            odom_msg=odom_msg,
            event_msg=event_msg,
        )

        return command_msg

    def publish_command(self):
        msg = self.make_command_msg()
        self.command_pub.publish(msg)

    def stop_motion(self):
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.current_event = "stop"
        self.publish_command()

    def timer_callback(self):
        keys = get_pending_keys()

        if keys:
            if "q" in keys:
                self.stop_motion()
                self.get_logger().info("Quit requested.")
                rclpy.shutdown()
                return

            self.handle_key(keys[-1])

        self.update_pose()
        self.publish_command()


def main():
    os.environ["ROS_DOMAIN_ID"] = ROS_DOMAIN_ID

    old_settings = termios.tcgetattr(sys.stdin)
    node = None

    try:
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init()
        node = KeyboardCommandUserTest()

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            if rclpy.ok():
                node.stop_motion()

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        print("\nKeyboard CommandUser test stopped.")


if __name__ == "__main__":
    main()