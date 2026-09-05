#!/usr/bin/env python3
# ROS 2 Humble / Python3
import math, os, csv, yaml
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float32

def yaw_from_quat(q):
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def rot_rect_contains(wx, wy, cx, cy, yaw, w, h):
    """회전 직사각형 포함 여부: world 점(wx,wy) ∈ Rect(center=cx,cy,yaw,w,h)?"""
    dx = wx - cx
    dy = wy - cy
    c = math.cos(-yaw); s = math.sin(-yaw)
    lx = c*dx - s*dy
    ly = s*dx + c*dy
    return (abs(lx) <= w*0.5) and (abs(ly) <= h*0.5)

class RelSpeedNode(Node):
    def __init__(self):
        super().__init__('speed_only_from_lidar')

        # ===== 파라미터 =====
        self.declare_parameter('front_fov_deg', 90.0)      # 전방 FOV (±35도)
        self.declare_parameter('opp_lane_y_min', -0.5)     # ROI y-band
        self.declare_parameter('opp_lane_y_max',  0.5)
        self.declare_parameter('min_range', 0.08)
        self.declare_parameter('max_range', 12.0)

        # ROI 안에서 빔 선택 방법
        #  - 가장 가까운 빔 기준으로 ±local_half 인덱스 모아서
        #  - 그 안에서 range 중앙값 사용
        self.declare_parameter('local_half', 2)            # 인덱스 범위 ±2 → 최대 5개
        # (원래 use_centroid 쓰던 것 대신, 지금은 local median 방식 사용)

        # 속도/스무딩
        self.declare_parameter('vel_smoothing', 5)         # 시간창 개수 (3,5 등)
        self.declare_parameter('use_speed_median', False)  # False: mean, True: median
        self.declare_parameter('max_reasonable_speed', 8.0) # m/s (속도 스파이크 컷)
        self.declare_parameter('vel_dt_min', 0.2)          # 속도 계산 최소 ΔT [s]

        # TTC
        self.declare_parameter('ttc_min_speed', 0.1)      # 너무 느리면 TTC=inf
        self.declare_parameter('ttc_max_cap', 99.9)

        # 콘솔/로그
        self.declare_parameter('print_hz', 5.0)
        self.declare_parameter('print_header_every', 30)
        self.declare_parameter('log_csv_path', '')
        self.declare_parameter('log_flush_every', 1)

        # 위험구역
        self.declare_parameter('risk_enable', True)
        self.declare_parameter('risk_area_yaml', '')
        self.declare_parameter('risk_frame', 'map')
        self.declare_parameter('risk_gate_mode', 'ego')    # ego | target | both

        # 포즈/속도 소스
        self.declare_parameter('use_amcl_pose', True)
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # ===== 파라미터 로드 =====
        self.fov = math.radians(float(self.get_parameter('front_fov_deg').value))
        self.ymin = float(self.get_parameter('opp_lane_y_min').value)
        self.ymax = float(self.get_parameter('opp_lane_y_max').value)
        self.rmin = float(self.get_parameter('min_range').value)
        self.rmax = float(self.get_parameter('max_range').value)

        self.local_half = int(self.get_parameter('local_half').value)

        self.win  = int(self.get_parameter('vel_smoothing').value)
        self.use_speed_median = bool(self.get_parameter('use_speed_median').value)
        self.vmax_gate = float(self.get_parameter('max_reasonable_speed').value)
        self.vel_dt_min = float(self.get_parameter('vel_dt_min').value)

        self.ttc_eps = float(self.get_parameter('ttc_min_speed').value)
        self.ttc_cap = float(self.get_parameter('ttc_max_cap').value)

        self.print_period = 1.0 / max(1e-6, float(self.get_parameter('print_hz').value))
        self.last_print_t = None
        self.print_count = 0
        self.print_header_every = int(self.get_parameter('print_header_every').value)

        self.risk_enable = bool(self.get_parameter('risk_enable').value)
        self.risk_yaml = str(self.get_parameter('risk_area_yaml').value)
        self.risk_frame = str(self.get_parameter('risk_frame').value)
        self.risk_gate_mode = str(self.get_parameter('risk_gate_mode').value).lower().strip()
        self.risk_rects = []  # (cx,cy,yaw,w,h,name)

        self.use_amcl = bool(self.get_parameter('use_amcl_pose').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        # ===== IO =====
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 20)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 50)
        if self.use_amcl:
            self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 30)

        self.pub_vrel_vec       = self.create_publisher(Twist,  '/target_rel_vel',    10)
        self.pub_vrel_speed     = self.create_publisher(Float32,'/target_rel_speed',  10)
        self.pub_range          = self.create_publisher(Float32,'/target_range',      10)
        self.pub_target_speed   = self.create_publisher(Float32,'/target_speed_est',  10)
        self.pub_ttc_anywhere   = self.create_publisher(Float32,'/target_ttc',        10)
        self.pub_ttc_in_risk    = self.create_publisher(Float32,'/target_ttc_real',   10)

        # ===== 상태 =====
        self.pose_now = None
        self.ego_vx = 0.0
        self.ego_vy = 0.0

        # ΔT 표본
        self.sample_t = None
        self.sample_px = None
        self.sample_py = None
        self.sample_pose = None  # (x,y,yaw) at sample_t

        # 거리/표적 유지용
        self.prev_scan_t = None
        self.prev_pose   = None
        self.prev_target = None

        # 속도 스무딩 버퍼
        self.speed_hist = deque(maxlen=self.win)

        # ===== CSV =====
        self._log_path = str(self.get_parameter('log_csv_path').value)
        self._log_flush_every = max(1, int(self.get_parameter('log_flush_every').value))
        self._log_f = None
        self._log_w = None
        self._log_count = 0
        if self._log_path:
            try:
                d = os.path.dirname(self._log_path)
                if d:
                    os.makedirs(d, exist_ok=True)
                new_file = (not os.path.exists(self._log_path)
                            or os.stat(self._log_path).st_size == 0)
                self._log_f = open(self._log_path, 'a', newline='')
                self._log_w = csv.writer(self._log_f)
                if new_file:
                    # 간단한 컬럼 구성
                    self._log_w.writerow([
                        'time','dt_used','ego_speed',
                        'vrel_speed','range','ttc','ttc_real'
                    ])
                    self._log_f.flush()
                self.get_logger().info(f'[RelSpeed] CSV logging -> {self._log_path}')
            except Exception as e:
                self.get_logger().error(f'CSV open failed: {self._log_path} ({e})')

        # ===== 위험구역 로드 =====
        if self.risk_enable and self.risk_yaml:
            try:
                with open(self.risk_yaml, 'r') as f:
                    data = yaml.safe_load(f) or {}
                if isinstance(data, str):
                    data = yaml.safe_load(data) or {}
                viz = data.get('risk_area_viz') or {}
                if isinstance(viz, str):
                    viz = yaml.safe_load(viz) or {}
                params = viz.get('ros__parameters') or {}
                if isinstance(params, str):
                    params = yaml.safe_load(params) or {}
                rects = params.get('risk_rects', [])
                if isinstance(rects, str):
                    try:
                        rects = yaml.safe_load(rects) or []
                    except Exception:
                        rects = []
                if isinstance(rects, dict):
                    rects = [rects]
                if not isinstance(rects, list):
                    rects = []
                for rr in rects:
                    if not isinstance(rr, dict):
                        continue
                    center = rr.get('center', [0.0, 0.0])
                    if not (isinstance(center, (list, tuple)) and len(center) == 2):
                        center = [0.0, 0.0]
                    cx, cy = float(center[0]), float(center[1])
                    yaw_deg = float(rr.get('yaw_deg', 0.0))
                    w = float(rr.get('width', 0.0))
                    h = float(rr.get('height', 0.0))
                    name = rr.get('name', '')
                    self.risk_rects.append(
                        (cx, cy, math.radians(yaw_deg), w, h, name)
                    )
                self.get_logger().info(
                    f'[RelSpeed] Loaded {len(self.risk_rects)} risk rect(s) from {self.risk_yaml}'
                )
            except Exception as e:
                self.get_logger().error(f'[RelSpeed] risk_area_yaml load failed: {e}')
                self.risk_rects = []

        self.get_logger().info(
            '[RelSpeed] TTC(anywhere) + TTC(real in risk) with ΔT-based velocity '
            f'(vel_smoothing={self.win}, use_speed_median={self.use_speed_median})'
        )

    # ===== 유틸 =====
    def destroy_node(self):
        try:
            if self._log_f:
                self._log_f.flush()
                self._log_f.close()
        except Exception:
            pass
        super().destroy_node()

    def odom_cb(self, m: Odometry):
        self.ego_vx = float(m.twist.twist.linear.x)
        self.ego_vy = float(m.twist.twist.linear.y)
        if not self.use_amcl:
            p = m.pose.pose
            self.pose_now = (p.position.x,
                             p.position.y,
                             yaw_from_quat(p.orientation))

    def amcl_cb(self, m: PoseWithCovarianceStamped):
        p = m.pose.pose
        self.pose_now = (p.position.x,
                         p.position.y,
                         yaw_from_quat(p.orientation))

    def _maybe_print(self, t, ego_speed, vrel_speed, rng, ttc_any, ttc_real, tag=''):
        if self.last_print_t is None or (t - self.last_print_t) >= self.print_period:
            self.last_print_t = t
            self.print_count += 1
            if (self.print_count % max(1, self.print_header_every)) == 1:
                print("   t[s]  | ego[m/s] | v_rel[m/s] | range[m] | TTC(any) | TTC(real) | note")
            sa = "inf" if math.isinf(ttc_any)  else f"{ttc_any:6.2f}"
            sr = "inf" if math.isinf(ttc_real) else f"{ttc_real:6.2f}"
            print(f"{t:7.2f} | {ego_speed:8.3f} | {vrel_speed:10.3f} | "
                  f"{rng:8.3f} | {sa} | {sr} | {tag}")

    def _in_any_risk_point(self, wx, wy):
        if not (self.risk_enable and self.risk_rects):
            return True
        for (cx, cy, yaw, w, h, _name) in self.risk_rects:
            if rot_rect_contains(wx, wy, cx, cy, yaw, w, h):
                return True
        return False

    def _update_speed_hist_and_get(self, vmag_raw: float) -> float:
        """새로운 속도 샘플을 히스토리에 반영하고 mean/median 반환."""
        if vmag_raw <= self.vmax_gate:
            self.speed_hist.append(vmag_raw)
        if not self.speed_hist:
            return vmag_raw
        vals = list(self.speed_hist)
        if self.use_speed_median:
            vals.sort()
            n = len(vals)
            mid = n // 2
            if n % 2 == 1:
                return vals[mid]
            else:
                return 0.5 * (vals[mid-1] + vals[mid])
        else:
            return sum(vals) / len(vals)

    def _current_smoothed_speed(self) -> float:
        """새 샘플 없이 현재 히스토리만으로 스무딩 속도 반환."""
        if not self.speed_hist:
            return 0.0
        vals = list(self.speed_hist)
        if self.use_speed_median:
            vals.sort()
            n = len(vals)
            mid = n // 2
            if n % 2 == 1:
                return vals[mid]
            else:
                return 0.5 * (vals[mid-1] + vals[mid])
        else:
            return sum(vals) / len(vals)

    # ===== 메인 콜백 =====
    def scan_cb(self, scan: LaserScan):
        if self.pose_now is None:
            return

        t = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        angle = scan.angle_min
        half = self.fov * 0.5

        # 1) ROI 안 빔 수집: (idx, r, x, y)
        roi_points = []
        idx = 0
        for r in scan.ranges:
            if math.isfinite(r) and self.rmin < r < self.rmax:
                th = angle
                if -half <= th <= half:
                    x = r * math.cos(th)
                    y = r * math.sin(th)
                    if self.ymin <= y <= self.ymax:
                        roi_points.append((idx, r, x, y))
            angle += scan.angle_increment
            idx += 1

        if not roi_points:
            # 타깃 없음 → 상태만 유지
            self.prev_target = None
            self.prev_pose = self.pose_now
            self.prev_scan_t = t
            return

        # 2) 가장 가까운 빔 찾고, 주변 ±local_half 안에서 range 중앙값 타깃 선택
        best = min(roi_points, key=lambda p: p[1])  # p[1] = r
        min_idx, min_r, min_x, min_y = best

        local_pts = [p for p in roi_points if abs(p[0] - min_idx) <= self.local_half]
        if local_pts:
            local_pts.sort(key=lambda p: p[1])  # range 기준 정렬
            mid = len(local_pts) // 2
            med_idx, med_r, med_x, med_y = local_pts[mid]
            target_now = (med_x, med_y)
            rng = med_r
        else:
            target_now = (min_x, min_y)
            rng = min_r

        # 거리 퍼블리시
        self.pub_range.publish(Float32(data=float(rng)))

        # 3) 월드 좌표 (게이팅용)
        (x_now, y_now, yaw_now) = self.pose_now
        bx, by = target_now
        wx = x_now + math.cos(yaw_now)*bx - math.sin(yaw_now)*by
        wy = y_now + math.sin(yaw_now)*bx + math.cos(yaw_now)*by

        ego_in = self._in_any_risk_point(x_now, y_now)
        tgt_in = self._in_any_risk_point(wx, wy)
        mode = self.risk_gate_mode
        if mode == 'ego':
            inside_risk = ego_in
        elif mode == 'target':
            inside_risk = tgt_in
        else:  # both
            inside_risk = ego_in and tgt_in

        # 4) ΔT 표본 초기화
        if self.sample_t is None:
            self.sample_t = t
            self.sample_pose = self.pose_now
            self.sample_px, self.sample_py = target_now
            self.prev_target = target_now
            self.prev_pose = self.pose_now
            self.prev_scan_t = t
            return

        # 5) ΔT 확인 및 상대속도 계산
        dT = t - self.sample_t
        vrel_x = 0.0
        vrel_y = 0.0
        vmag = 0.0
        speed_valid = False

        if dT >= self.vel_dt_min:
            (xs, ys, yaws) = self.sample_pose
            pbx, pby = self.sample_px, self.sample_py

            # 표본 시점 타깃을 world로
            wpx = xs + math.cos(yaws)*pbx - math.sin(yaws)*pby
            wpy = ys + math.sin(yaws)*pbx + math.cos(yaws)*pby

            # world → 현재 base_link
            wpx -= x_now
            wpy -= y_now
            c = math.cos(-yaw_now); s = math.sin(-yaw_now)
            px = c*wpx - s*wpy
            py = s*wpx + c*wpy

            # ΔT 동안 상대변위
            vrel_x = (target_now[0] - px) / dT
            vrel_y = (target_now[1] - py) / dT
            vmag = math.hypot(vrel_x, vrel_y)

            # 스무딩 업데이트
            vmag_sm = self._update_speed_hist_and_get(vmag)

            # 퍼블리시
            msg_vec = Twist()
            msg_vec.linear.x = vrel_x
            msg_vec.linear.y = vrel_y
            self.pub_vrel_vec.publish(msg_vec)
            self.pub_vrel_speed.publish(Float32(data=float(vmag_sm)))

            speed_valid = True

            # 표본 롤링
            self.sample_t = t
            self.sample_pose = self.pose_now
            self.sample_px, self.sample_py = target_now
        else:
            # 아직 ΔT 모자람 → 기존 히스토리만 사용
            vmag_sm = self._current_smoothed_speed()

        # 6) TTC(anywhere)
        if speed_valid and vmag_sm > self.ttc_eps and rng > 1e-6:
            ttc_any = min(rng / vmag_sm, self.ttc_cap)
        else:
            ttc_any = float('inf')
        self.pub_ttc_anywhere.publish(
            Float32(data=self.ttc_cap if math.isinf(ttc_any) else ttc_any)
        )

        # 7) TTC(real in risk)
        if inside_risk and speed_valid and vmag_sm > self.ttc_eps and rng > 1e-6:
            ttc_real = min(rng / vmag_sm, self.ttc_cap)
        else:
            ttc_real = float('inf')
        self.pub_ttc_in_risk.publish(
            Float32(data=self.ttc_cap if math.isinf(ttc_real) else ttc_real)
        )

        # 8) ego/target speed 추정
        ego_speed = math.hypot(self.ego_vx, self.ego_vy)
        target_speed_est = math.hypot(vrel_x + self.ego_vx,
                                      vrel_y + self.ego_vy)
        self.pub_target_speed.publish(Float32(data=float(target_speed_est)))

        # 9) 콘솔/CSV 로그
        tag = 'IN' if inside_risk else 'OUT'
        self._maybe_print(t, ego_speed, vmag_sm, rng, ttc_any, ttc_real, tag=tag)

        if self._log_w and speed_valid:
            self._log_w.writerow([
                f'{t:.3f}', f'{dT:.3f}', f'{ego_speed:.3f}',
                f'{vmag_sm:.3f}', f'{rng:.3f}',
                ('inf' if math.isinf(ttc_any)  else f'{ttc_any:.3f}'),
                ('inf' if math.isinf(ttc_real) else f'{ttc_real:.3f}')
            ])
            self._log_count += 1
            if (self._log_count % self._log_flush_every) == 0:
                try:
                    self._log_f.flush()
                except Exception:
                    pass

        # 10) 상태 유지
        self.prev_target = target_now
        self.prev_pose   = self.pose_now
        self.prev_scan_t = t

def main():
    rclpy.init()
    node = RelSpeedNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
