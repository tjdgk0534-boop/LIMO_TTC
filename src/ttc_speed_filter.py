#!/usr/bin/env python3
# ROS 2 Humble / Python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class TTCSpeedFilter(Node):
    def __init__(self):
        super().__init__('ttc_speed_filter')

        # ===== 파라미터 선언 =====
        # 토픽 이름
        self.declare_parameter('cmd_in_topic',  '/cmd_vel_smooth')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')
        self.declare_parameter('ttc_topic',     '/target_ttc_real')

        # TTC 임계값
        #  - ttc <= ttc_stop  → 정지(한 번만)
        self.declare_parameter('ttc_stop',      5.0)    # [s]

        # TTC 유효 범위
        self.declare_parameter('ttc_valid_max',   20.0)   # 이 값 초과는 inf 취급

        # 정지 유지 시간
        self.declare_parameter('stop_hold_time',  2.0)    # STOP 유지 시간 [s]

        # 디버그 로그 주기
        self.declare_parameter('state_print_period', 0.5) # [s]

        # ===== 파라미터 로드 =====
        self.cmd_in_topic   = self.get_parameter(
            'cmd_in_topic').get_parameter_value().string_value
        self.cmd_out_topic  = self.get_parameter(
            'cmd_out_topic').get_parameter_value().string_value
        self.ttc_topic      = self.get_parameter(
            'ttc_topic').get_parameter_value().string_value

        self.ttc_stop       = self.get_parameter(
            'ttc_stop').get_parameter_value().double_value
        self.ttc_valid_max  = self.get_parameter(
            'ttc_valid_max').get_parameter_value().double_value
        self.stop_hold      = self.get_parameter(
            'stop_hold_time').get_parameter_value().double_value
        self.print_period   = self.get_parameter(
            'state_print_period').get_parameter_value().double_value

        # ===== 상태 변수 =====
        # 최신 TTC 값
        self.ttc_real = float('inf')

        # 한 번이라도 STOP을 발동했는지 여부 (원샷)
        self.stopped_once = False

        # 현재 STOP 상태인지
        self.in_stop = False
        self.stop_start_t = None

        # 로그 출력 타이밍
        self.last_print_t = None

        # ===== 통신 설정 =====
        self.sub_cmd_in = self.create_subscription(
            Twist,
            self.cmd_in_topic,
            self.cmd_in_cb,
            10
        )
        self.sub_ttc = self.create_subscription(
            Float32,
            self.ttc_topic,
            self.ttc_cb,
            10
        )
        self.pub_cmd_out = self.create_publisher(
            Twist,
            self.cmd_out_topic,
            10
        )

        self.get_logger().info(
            f'[TTCFilter] ONE-SHOT mode\n'
            f'  cmd_in={self.cmd_in_topic}, cmd_out={self.cmd_out_topic}, '
            f'ttc_topic={self.ttc_topic}\n'
            f'  ttc_stop={self.ttc_stop:.2f} s, stop_hold_time={self.stop_hold:.2f} s\n'
            f'  ttc_valid_max={self.ttc_valid_max:.1f}'
        )

    # ===== TTC 콜백 =====
    def ttc_cb(self, msg: Float32):
        raw = float(msg.data)

        # 비정상 값 / 너무 큰 값은 inf 취급
        if raw <= 0.0 or raw > self.ttc_valid_max or math.isnan(raw):
            self.ttc_real = float('inf')
        else:
            self.ttc_real = raw

    # ===== cmd_vel 필터링 콜백 =====
    def cmd_in_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        ttc = self.ttc_real

        # 기본 출력 = 입력 cmd_vel 복사
        out = Twist()
        out.linear.x  = msg.linear.x
        out.linear.y  = msg.linear.y
        out.linear.z  = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        # 아직 한 번도 STOP을 쓴 적이 없고, 현재도 STOP이 아닐 때만
        if (not self.stopped_once) and (not self.in_stop):
            # TTC가 유효하고, 임계값보다 작으면 → STOP 발동
            if math.isfinite(ttc) and ttc <= self.ttc_stop:
                self.in_stop = True
                self.stopped_once = True
                self.stop_start_t = now
                self.get_logger().info(
                    f'[TTCFilter] LATCH STOP! ttc={ttc:.2f} <= {self.ttc_stop:.2f}'
                )

        # STOP 상태에서는 일정 시간 동안 무조건 정지
        if self.in_stop:
            elapsed = now - self.stop_start_t if self.stop_start_t is not None else 0.0

            # 정지 유지 시간 동안은 계속 0으로
            if elapsed < self.stop_hold:
                out.linear.x  = 0.0
                out.linear.y  = 0.0
                out.linear.z  = 0.0
                out.angular.x = 0.0
                out.angular.y = 0.0
                out.angular.z = 0.0
            else:
                # 정지 유지 시간 끝나면 → 다시 입력 속도 그대로 통과
                self.in_stop = False
                self.get_logger().info(
                    f'[TTCFilter] RELEASE after {elapsed:.2f}s, '
                    f'filter now transparent (no further STOP).'
                )

        # 최종 명령 퍼블리시
        self.pub_cmd_out.publish(out)

        # ===== 디버그 출력 =====
        if self.print_period > 0.0:
            if (self.last_print_t is None) or (now - self.last_print_t >= self.print_period):
                self.last_print_t = now
                vx_in  = msg.linear.x
                vx_out = out.linear.x
                ttc_str = 'inf' if not math.isfinite(ttc) else f'{ttc:.2f}'
                state = 'STOP' if self.in_stop else ('USED' if self.stopped_once else 'READY')
                self.get_logger().info(
                    f'[TTCFilter] state={state}  '
                    f'ttc={ttc_str}  '
                    f'vx_in={vx_in:.2f}  vx_out={vx_out:.2f}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = TTCSpeedFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
