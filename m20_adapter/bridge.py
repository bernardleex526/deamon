"""ROS input to documented basic_server Cmd=25. Dry-run never opens a socket."""
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import SetBool

from m20_adapter.core import GAITS, Guard, UdpClient, inspect_scan, velocity_items
from m20_adapter.operator_control import OperatorControl
from m20_adapter.operator_server import OperatorServer


class Bridge(Node):
    def __init__(self):
        super().__init__('m20_bridge')
        defaults = dict(dry_run=True, commissioned=False, aos_host='10.21.31.103',
                        aos_port=30000, max_vx=0.3, max_vy=0.3, max_wz=0.6,
                        scan_frame='lidar_link', stop_front=0.7, stop_back=0.7,
                         stop_half_width=0.4, enable_operator=False,
                         operator_token_file='', operator_page='',
                         operator_host='0.0.0.0', operator_port=8080, android_port=8889)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.dry_run = p('dry_run')
        self.commissioned = p('commissioned')
        if not self.dry_run and not self.commissioned:
            raise RuntimeError('live transport requires commissioned:=true')
        if self.get_parameter('use_sim_time').value and not self.dry_run:
            raise RuntimeError('live transport cannot use simulation time')
        self.guard = Guard(limits=(p('max_vx'), p('max_vy'), p('max_wz')))
        self.scan_frame = p('scan_frame')
        self.stop = dict(stop_front=p('stop_front'), stop_back=p('stop_back'),
                         stop_half_width=p('stop_half_width'))
        import math
        if any(not math.isfinite(v) or v <= 0 for v in self.stop.values()):
            raise ValueError('stop dimensions must be positive and finite')
        self.client = None if self.dry_run else UdpClient(p('aos_host'), p('aos_port'))
        self.last_query = -float('inf')
        self.operator = None
        self.server = None
        self.follow_key = None
        self.follow_ready = False
        self.follow_started = -float('inf')
        self.target_pub = self.create_publisher(Float64MultiArray, '/robot_nexus/operator_target', 1)
        self.create_subscription(Float64MultiArray, '/robot_nexus/operator_velocity', self.tagged_command, 1)
        self.scan_preview = []
        if p('enable_operator'):
            token_file = p('operator_token_file') or os.environ.get('M20_OPERATOR_TOKEN_FILE', '')
            if not token_file:
                raise ValueError('set M20_OPERATOR_TOKEN_FILE or use scripts/m20ctl preview')
            token = Path(token_file).read_text().strip()
            self.operator = OperatorControl(self.guard, preview=self.dry_run)
            self.server = OperatorServer(token, Path(p('operator_page')).read_bytes(),
                                         p('operator_host'), p('operator_port'), p('android_port'))
            self.server.start()
        self.preview = self.create_publisher(Twist, '/m20/cmd_vel_guarded', 1)
        self.status_pub = self.create_publisher(String, '/m20/bridge_status', 1)
        self.create_subscription(Twist, '/m20/cmd_vel_raw', self.command, 1)
        self.create_subscription(LaserScan, '/m20/scan', self.scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/m20/arm', self.arm)
        if self.dry_run:
            self.create_subscription(String, '/m20/mock_basic_status', self.mock_status, 1)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def command(self, msg):
        velocity = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not self.operator:
            self.guard.update_command(velocity)

    def tagged_command(self, msg):
        if (self.operator and self.operator.mode == 'follow' and len(msg.data) == 4
                and msg.data[0] == self.operator.session):
            self.follow_ready = True
            self.operator.follow_command(tuple(msg.data[1:]))

    def scan(self, msg):
        valid, clear = inspect_scan(msg, self.get_clock().now().nanoseconds / 1e9, **self.stop)
        self.guard.update_scan(valid and msg.header.frame_id == self.scan_frame, clear)
        if self.operator:
            import math
            self.scan_preview = [
                [distance * math.cos(msg.angle_min + i * msg.angle_increment),
                 distance * math.sin(msg.angle_min + i * msg.angle_increment)]
                for i, distance in enumerate(msg.ranges)
                if i % 4 == 0 and math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max]

    def arm(self, request, response):
        if self.operator and request.data:
            response.success = False
            response.message = 'Use the operator start button (exclusive control lease)'
            return response
        if self.operator:
            self.operator.stop('ROS operator stopped')
        if request.data:
            response.success = self.guard.arm()
        else:
            self.guard.disarm('operator disarmed')
            response.success = True
        response.message = self.guard.reason
        return response

    def mock_status(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError('status must be an object')
            self.guard.update_status(value)
        except ValueError:
            self.guard.disarm('invalid mock status')

    def tick(self):
        try:
            if self.client:
                for _, value in self.client.receive():
                    items = value['Items']
                    if items.get('ErrorCode', 0) != 0:
                        self.guard.disarm('basic_server rejected command: ' + str(items))
                    elif value['Type'] == 1002 and value['Command'] == 6:
                        # Query ACKs are not status and must not refresh freshness.
                        if 'BasicStatus' in items or 'MotionState' in items:
                            self.guard.update_status(items)
                if time.monotonic() - self.last_query >= 1.0:
                    self.client.heartbeat()
                    self.last_query = time.monotonic()
            if self.operator:
                self.operator.tick()
                self.server.drain(self.operator)
                self.sync_follow()
            velocity = self.guard.output()
            if self.operator:
                self.operator.tick()
            if self.client:
                self.client.send(2, 25, velocity_items(velocity))
        except OSError as exc:
            self.guard.disarm('UDP failure: ' + str(exc))
            velocity = (0.0, 0.0, 0.0)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = velocity
        self.preview.publish(msg)
        status = String()
        status.data = json.dumps(dict(dry_run=self.dry_run, armed=self.guard.armed,
                                      reason=self.guard.reason, velocity=velocity))
        self.status_pub.publish(status)
        if self.server:
            minimums = GAITS.get(self.guard.status.get('Gait'), (0., 0., 0.) if self.dry_run else None)
            speeds = [min(cap, max(low, preferred)) if low <= cap else 0.
                      for cap, low, preferred in zip(self.guard.limits, minimums, (.2, .25, .4))] if minimums else [0., 0., 0.]
            self.server.update(dict(ready=True, dry_run=self.dry_run,
                                    armed=self.guard.armed, reason=self.guard.reason,
                                    velocity=velocity, operator=self.operator.snapshot(),
                                    scan=self.scan_preview,
                                    follow_ready=self.follow_ready,
                                    control_speeds=speeds,
                                    speed_limits=self.guard.limits,
                                    gait_minimums=minimums,
                                    robot_status=self.guard.status))

    def sync_follow(self):
        key = self.operator.session if self.operator.mode == 'follow' else None
        if key != self.follow_key:
            self.follow_key = key
            self.follow_ready = False
            self.follow_started = time.monotonic()
            target = self.operator.target if key else (0., 0.)
            msg = Float64MultiArray()
            msg.data = [float(key or 0), float(target[0]), float(target[1])]
            self.target_pub.publish(msg)
        if key and not self.follow_ready:
            self.guard.update_command((0., 0., 0.))
            if time.monotonic() - self.follow_started > 1.0:
                self.operator.stop('follow node did not acknowledge the selected target')

    def close(self):
        if self.operator:
            self.operator.stop('bridge shutting down')
        if self.client:
            try:
                self.client.send(2, 25, velocity_items((0.0, 0.0, 0.0)))
            except OSError:
                pass
            self.client.close()
        if self.server:
            self.server.close()


def main():
    rclpy.init()
    node = None
    try:
        node = Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
