#!/usr/bin/env python3
"""
Live PX4 MAVLink viewer using PyImGui (imgui[full] + GLFW + OpenGL).
No ROS, no numpy/matplotlib.

Features:
- ARM64 Linux friendly (GPU accelerated via OpenGL).
- Elastic layout with left control panel and right canvas.
- 1:1 XY rendering with metric grid and world axes.
- Pan (left-drag on canvas) and zoom (mouse wheel on canvas).
- Circle expected trajectory render (orange).
- Anchor transform (x, y, yaw).
- Actual trail fading.
- Real-time vehicle yaw arrow (Red).
- Target source toggle.
- Target setpoint + yaw arrow (GREEN).
"""

from __future__ import annotations

import argparse
import math
import threading
import time
import sys
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

try:
    import numpy  # noqa: F401
except Exception:
    print("plot_live.py requires numpy. Install with: python3 -m pip install numpy")
    sys.exit(1)

import glfw
import OpenGL.GL as gl
import imgui
from imgui.integrations.glfw import GlfwRenderer
from pymavlink import mavutil


@dataclass
class CircleConfig:
    radius: float = 2.0
    speed: float = 1.0
    ramp: float = 1.0


@dataclass
class TargetSetpoint:
    x: float
    y: float
    yaw: float
    speed: float
    timestamp: float


class LiveViewerImGui:
    def __init__(self, link: str, rate_hz: float):
        self.link = link
        self.rate_hz = max(1.0, rate_hz)
        self.running = True

        # --- 视图状态 (世界坐标 <-> 屏幕坐标) ---
        self.px_per_meter = 60.0
        self.center_x = 0.0
        self.center_y = 0.0

        # --- 配置参数 ---
        self.circle = CircleConfig()
        self.expected_polyline: List[Tuple[float, float]] = []

        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_yaw = 0.0

        self.fade_start_s = 2.0
        self.fade_end_s = self.fade_start_s + 2.0
        self.show_full_trail = False

        self.tangent_fit_window_s = 0.45
        self.tangent_lead_s = 0.18
        self.tangent_min_speed_mps = 0.12
        self._tan_valid = False
        self._tan_yaw_unwrapped = 0.0
        self._tan_omega = 0.0
        self._tan_last_update = 0.0

        # 0: fitted_tangent, 1: trajectory_setpoint
        self.target_source_idx = 0
        self.target_source_items = ["fitted_tangent", "trajectory_setpoint"]

        # --- 实时数据 ---
        self.lock = threading.Lock()
        self.trail: Deque[Tuple[float, float, float]] = deque(maxlen=20000)
        self.latest_pos: Optional[Tuple[float, float]] = None
        self.latest_yaw: Optional[float] = None
        self.latest_target: Optional[TargetSetpoint] = None
        self.last_msg_time = 0.0
        self.msg_count = 0
        self.target_msg_count = 0
        self.connection_state = "Connecting..."

        self._recompute_expected()

        self.thread = threading.Thread(target=self._mav_worker, daemon=True)
        self.thread.start()

    # --- 数学与轨迹计算 (保持不变) ---

    @staticmethod
    def _circle_xy_at_time(t: float, radius: float, speed: float, ramp: float) -> Tuple[float, float]:
        t = max(0.0, t)
        sgn = 1.0 if speed >= 0.0 else -1.0
        v_abs = abs(speed)
        if ramp > 0.0:
            if t < ramp:
                s = 0.5 * v_abs * t * t / ramp
            else:
                s = 0.5 * v_abs * ramp + v_abs * (t - ramp)
        else:
            s = v_abs * t
        theta = sgn * s / max(radius, 1e-6)
        # Keep t=0 at the geometric right-most point of the circle.
        x = radius * (math.cos(theta) - 1.0)
        y = radius * math.sin(theta)
        return x, y

    def _recompute_expected(self):
        r = self.circle.radius
        v = self.circle.speed
        ramp = self.circle.ramp
        v_abs = abs(v)
        period = 2.0 * math.pi * r / (v_abs if v_abs > 0.05 else 0.05)
        horizon = max(8.0, ramp + 1.25 * period)

        c = math.cos(self.anchor_yaw)
        s = math.sin(self.anchor_yaw)
        pts: List[Tuple[float, float]] = []
        n = 500
        for i in range(n + 1):
            t = horizon * i / n
            x_local, y_local = self._circle_xy_at_time(t, r, v, ramp)

            x_world = self.anchor_x + c * x_local + s * y_local
            y_world = self.anchor_y + s * x_local - c * y_local
            pts.append((x_world, y_world))
        self.expected_polyline = pts

    @staticmethod
    def _solve3(m, b):
        a = [m[0][:] + [b[0]], m[1][:] + [b[1]], m[2][:] + [b[2]]]
        for i in range(3):
            pivot = i
            for r in range(i + 1, 3):
                if abs(a[r][i]) > abs(a[pivot][i]): pivot = r
            if abs(a[pivot][i]) < 1e-12: return None
            if pivot != i: a[i], a[pivot] = a[pivot], a[i]
            inv = 1.0 / a[i][i]
            for c in range(i, 4): a[i][c] *= inv
            for r in range(3):
                if r == i: continue
                f = a[r][i]
                if f == 0.0: continue
                for c in range(i, 4): a[r][c] -= f * a[i][c]
        return [a[0][3], a[1][3], a[2][3]]

    def _estimate_tangent_yaw_from_history(self, points, now):
        cutoff = now - self.tangent_fit_window_s
        recent = []
        for t, x, y in reversed(points):
            if t < cutoff: break
            recent.append((t, x, y))
        recent.reverse()

        if len(recent) < 6: return None

        s0=s1=s2=s3=s4=bx0=bx1=bx2=by0=by1=by2=0.0
        for t, x, y in recent:
            tau = t - now
            tau2 = tau * tau
            s0+=1.0; s1+=tau; s2+=tau2; s3+=tau2*tau; s4+=tau2*tau2
            bx0+=x; bx1+=x*tau; bx2+=x*tau2
            by0+=y; by1+=y*tau; by2+=y*tau2

        mat = [[s0, s1, s2], [s1, s2, s3], [s2, s3, s4]]
        ax = self._solve3(mat, [bx0, bx1, bx2])
        ay = self._solve3(mat, [by0, by1, by2])
        if ax is None or ay is None: return None

        vx, vy = ax[1], ay[1]
        speed = math.hypot(vx, vy)
        if speed < self.tangent_min_speed_mps: return None

        yaw_raw = math.atan2(vy, vx)
        if not self._tan_valid:
            self._tan_valid=True; self._tan_yaw_unwrapped=yaw_raw; self._tan_omega=0.0; self._tan_last_update=now
            return (yaw_raw + math.pi) % (2.0*math.pi) - math.pi

        ref = self._tan_yaw_unwrapped
        unwrapped = ref + ((yaw_raw - ref + math.pi) % (2.0*math.pi) - math.pi)

        dt = max(1e-3, now - self._tan_last_update)
        omega_raw = (unwrapped - self._tan_yaw_unwrapped) / dt
        alpha_omega = math.exp(-dt / 0.25)
        alpha_yaw = math.exp(-dt / 0.15)
        self._tan_omega = alpha_omega * self._tan_omega + (1.0 - alpha_omega) * omega_raw
        self._tan_yaw_unwrapped = alpha_yaw * self._tan_yaw_unwrapped + (1.0 - alpha_yaw) * unwrapped
        self._tan_last_update = now

        yaw_lead = self._tan_yaw_unwrapped + self.tangent_lead_s * self._tan_omega
        return (yaw_lead + math.pi) % (2.0*math.pi) - math.pi

    # --- MAVLink 后台接收 ---

    def _mav_worker(self) -> None:
        try:
            m = mavutil.mavlink_connection(self.link)
            self.connection_state = "Waiting heartbeat..."
            m.wait_heartbeat(timeout=30)
            self.connection_state = f"Connected (Sys={m.target_system})"

            interval_us = int(1e6 / self.rate_hz)
            for msg_id in [mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                           mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                           mavutil.mavlink.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED]:
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, interval_us, 0, 0, 0, 0, 0
                )

            while self.running:
                msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE", "POSITION_TARGET_LOCAL_NED"], blocking=True, timeout=1.0)
                if msg is None: continue

                now = time.time()
                mtype = msg.get_type()

                with self.lock:
                    if mtype == "ATTITUDE":
                        self.latest_yaw = float(msg.yaw)
                    elif mtype == "POSITION_TARGET_LOCAL_NED":
                        type_mask = int(msg.type_mask)
                        x_ignored = bool(type_mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE)
                        y_ignored = bool(type_mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE)
                        vx_ignored = bool(type_mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE)
                        vy_ignored = bool(type_mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE)
                        yaw_ignored = bool(type_mask & mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE)

                        x = float(msg.x) if not x_ignored and math.isfinite(msg.x) else None
                        y = float(msg.y) if not y_ignored and math.isfinite(msg.y) else None
                        vx = float(msg.vx) if not vx_ignored and math.isfinite(msg.vx) else 0.0
                        vy = float(msg.vy) if not vy_ignored and math.isfinite(msg.vy) else 0.0
                        speed = math.hypot(vx, vy)
                        yaw = float(msg.yaw) if not yaw_ignored and math.isfinite(msg.yaw) else None
                        if yaw is None and speed > 1e-3: yaw = math.atan2(vy, vx)

                        if x is not None and y is not None and yaw is not None:
                            self.latest_target = TargetSetpoint(x=x, y=y, yaw=yaw, speed=speed, timestamp=now)
                            self.target_msg_count += 1
                    elif mtype == "LOCAL_POSITION_NED":
                        x, y = float(msg.x), float(msg.y)
                        self.trail.append((now, x, y))
                        self.latest_pos = (x, y)
                        self.last_msg_time = now
                        self.msg_count += 1

                        if not self.show_full_trail:
                            cutoff = now - (self.fade_end_s + 1.0)
                            while self.trail and self.trail[0][0] < cutoff:
                                self.trail.popleft()
        except Exception as e:
            self.connection_state = f"MAVLink Error: {e}"

    # --- 视图转换 ( ImGui 版本 ) ---

    def _w2s(self, wx: float, wy: float, canvas_p0: Tuple[float, float], canvas_size: Tuple[float, float]) -> Tuple[float, float]:
        """将世界坐标转换为 ImGui 画布的绝对屏幕坐标"""
        # Mirror X horizontally so screen direction matches current user expectation.
        sx = (self.center_x - wx) * self.px_per_meter + canvas_size[0] * 0.5 + canvas_p0[0]
        sy = canvas_size[1] * 0.5 - (wy - self.center_y) * self.px_per_meter + canvas_p0[1]
        return sx, sy

    def _s2w(self, sx: float, sy: float, canvas_p0: Tuple[float, float], canvas_size: Tuple[float, float]) -> Tuple[float, float]:
        """将屏幕绝对坐标转换回世界坐标"""
        wx = self.center_x - (sx - canvas_p0[0] - canvas_size[0] * 0.5) / self.px_per_meter
        wy = (canvas_size[1] * 0.5 - (sy - canvas_p0[1])) / self.px_per_meter + self.center_y
        return wx, wy

    # --- 绘制辅助 ---

    def _draw_arrow(self, draw_list, p1, p2, color, thickness=2.0, size=10.0):
        draw_list.add_line(*p1, *p2, color, thickness)
        angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        ap1 = (p2[0] - size * math.cos(angle - math.pi / 6), p2[1] - size * math.sin(angle - math.pi / 6))
        ap2 = (p2[0] - size * math.cos(angle + math.pi / 6), p2[1] - size * math.sin(angle + math.pi / 6))
        draw_list.add_triangle_filled(*p2, *ap1, *ap2, color)

    # --- 主渲染与交互逻辑 ---

    def _render_ui(self):
        # 强制主窗口占满整个真实系统窗口
        viewport_size = imgui.get_io().display_size
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(*viewport_size)

        # 开启主背景窗口
        flags = imgui.WINDOW_NO_DECORATION | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_BRING_TO_FRONT_ON_FOCUS
        imgui.begin("MainViewer", flags=flags)

        # ====== 1. 左侧控制面板 ======
        imgui.begin_child("Controls", width=320, border=True)

        imgui.text_colored("Connection", 0.0, 0.47, 0.8, 1.0)
        imgui.text_wrapped(f"Link: {self.link}")
        imgui.separator()
        imgui.spacing()

        imgui.text_colored("Trajectory Parameters", 1.0, 0.55, 0.0, 1.0)
        changed_r, r = imgui.drag_float("Radius (r)", self.circle.radius, 0.05, 0.1, 100.0, "%.2f m")
        changed_v, v = imgui.drag_float("Speed (v)", self.circle.speed, 0.05, -50.0, 50.0, "%.2f m/s")
        changed_ramp, ramp = imgui.drag_float("Ramp time", self.circle.ramp, 0.05, 0.0, 20.0, "%.2f s")
        if changed_r or changed_v or changed_ramp:
            self.circle = CircleConfig(radius=max(0.1, r), speed=v, ramp=max(0.0, ramp))
            self._recompute_expected()

        imgui.spacing()
        imgui.separator()
        imgui.text_colored("Anchor Transform", 1.0, 0.55, 0.0, 1.0)
        ch_ax, self.anchor_x = imgui.drag_float("Anchor X", self.anchor_x, 0.05, format="%.2f m")
        ch_ay, self.anchor_y = imgui.drag_float("Anchor Y", self.anchor_y, 0.05, format="%.2f m")
        ch_ayaw, self.anchor_yaw = imgui.drag_float("Anchor Yaw", self.anchor_yaw, 0.01, format="%.3f rad")
        if ch_ax or ch_ay or ch_ayaw:
            self._recompute_expected()

        imgui.spacing()
        imgui.separator()
        imgui.text_colored("Display Settings", 0.6, 0.6, 0.6, 1.0)
        ch_fade, self.fade_start_s = imgui.drag_float("Fade start", self.fade_start_s, 0.1, 0.1, 60.0, "%.1f s")
        if ch_fade: self.fade_end_s = self.fade_start_s + 2.0

        _, self.target_source_idx = imgui.combo("Target Source", self.target_source_idx, self.target_source_items)
        _, self.show_full_trail = imgui.checkbox("Show Full Trail", self.show_full_trail)

        imgui.spacing()
        imgui.separator()
        imgui.text_colored("NED Mapping (XY)", 0.6, 0.6, 0.6, 1.0)
        imgui.bullet_text("Horizontal: +X (North) -> LEFT, -X -> RIGHT (mirrored)")
        imgui.bullet_text("Vertical: +Y (East) -> UP, -Y -> DOWN")
        imgui.bullet_text("Z uses NED Down axis, but is not shown in this XY view")

        imgui.spacing()
        if imgui.button("Reset View (0,0)", width=-1):
            self.center_x = 0.0
            self.center_y = 0.0
            self.px_per_meter = 60.0

        imgui.spacing()
        imgui.separator()
        imgui.text_colored("Estimator Tuning", 0.6, 0.6, 0.6, 1.0)
        _, self.tangent_fit_window_s = imgui.drag_float("Fit window", self.tangent_fit_window_s, 0.05, 0.1, 5.0, "%.2f s")
        _, self.tangent_lead_s = imgui.drag_float("Lead time", self.tangent_lead_s, 0.01, 0.0, 2.0, "%.2f s")

        imgui.end_child()
        imgui.same_line()

        # ====== 2. 右侧画布区 ======
        imgui.begin_child("CanvasArea", border=True)

        now = time.time()

        # 准备状态文本
        with self.lock:
            msg_rate = self.msg_count / (now - self.thread.start_time) if hasattr(self.thread, 'start_time') else 0
            if not hasattr(self.thread, 'start_time') and self.msg_count > 0: self.thread.start_time = now
            n_pts = len(self.trail)
            age = (now - self.last_msg_time) if self.last_msg_time > 0 else float('inf')
            target = self.latest_target

        target_info = f"Target: ({target.x:.1f}, {target.y:.1f})" if target else "Target: N/A"
        status_text = (f"{self.connection_state} | Msgs: {self.msg_count} ({msg_rate:.1f}Hz) | Trail: {n_pts} | "
                       f"Last msg: {age:.2f}s | Scale: {self.px_per_meter:.1f} px/m | {target_info}")
        imgui.text_colored(status_text, 0.6, 0.6, 0.6, 1.0)

        # 获取画布绘图上下文
        p0 = imgui.get_cursor_screen_pos()
        avail = imgui.get_content_region_available()

        canvas_p0 = (float(p0[0]), float(p0[1]))
        canvas_size = (max(10.0, float(avail[0])), max(10.0, float(avail[1])))
        draw_list = imgui.get_window_draw_list()

        # 为了不超出画布，我们需要添加裁切框 (Clip Rect)
        canvas_p1 = (canvas_p0[0] + canvas_size[0], canvas_p0[1] + canvas_size[1])
        draw_list.push_clip_rect(canvas_p0[0], canvas_p0[1], canvas_p1[0], canvas_p1[1], intersect_with_current_clip_rect=True)

        # 画白色背景
        draw_list.add_rect_filled(canvas_p0[0], canvas_p0[1], canvas_p1[0], canvas_p1[1], imgui.get_color_u32_rgba(1, 1, 1, 1))

        # --- 交互：平移和缩放 ---
        # 我们使用 InvisibleButton 占满整个区域来捕获鼠标事件
        imgui.invisible_button("canvas_input", canvas_size[0], canvas_size[1])
        is_hovered = imgui.is_item_hovered()
        is_active = imgui.is_item_active()

        io = imgui.get_io()

        # 平移
        if is_active and imgui.is_mouse_dragging(0):
            delta = io.mouse_delta
            self.center_x += delta.x / self.px_per_meter
            self.center_y += delta.y / self.px_per_meter

        # 缩放
        if is_hovered and io.mouse_wheel != 0.0:
            mouse_pos = io.mouse_pos
            # 缩放前，鼠标位置对应的世界坐标
            wx0, wy0 = self._s2w(mouse_pos.x, mouse_pos.y, canvas_p0, canvas_size)

            factor = 1.1 if io.mouse_wheel > 0 else (1.0 / 1.1)
            self.px_per_meter = max(5.0, min(2000.0, self.px_per_meter * factor))

            # 缩放后，保持鼠标所在的世界坐标不变
            wx1, wy1 = self._s2w(mouse_pos.x, mouse_pos.y, canvas_p0, canvas_size)
            self.center_x += (wx0 - wx1)
            self.center_y += (wy0 - wy1)

        # --- 绘制网格 ---
        x_min, y_min = self._s2w(canvas_p0[0], canvas_p1[1], canvas_p0, canvas_size)
        x_max, y_max = self._s2w(canvas_p1[0], canvas_p0[1], canvas_p0, canvas_size)
        step = 1.0
        if step * self.px_per_meter < 15.0:
            step = float(max(1, int(math.ceil(15.0 / max(1e-6, step * self.px_per_meter)))))

        grid_col = imgui.get_color_u32_rgba(0.9, 0.9, 0.9, 1.0)
        x = math.floor(x_min / step) * step
        while x <= x_max:
            sx, _ = self._w2s(x, 0.0, canvas_p0, canvas_size)
            draw_list.add_line(sx, canvas_p0[1], sx, canvas_p1[1], grid_col, 1.0)
            x += step

        y = math.floor(y_min / step) * step
        while y <= y_max:
            _, sy = self._w2s(0.0, y, canvas_p0, canvas_size)
            draw_list.add_line(canvas_p0[0], sy, canvas_p1[0], sy, grid_col, 1.0)
            y += step

        axis_col = imgui.get_color_u32_rgba(0.6, 0.6, 0.6, 1.0)
        sx0, sy0 = self._w2s(0.0, 0.0, canvas_p0, canvas_size)
        draw_list.add_line(sx0, canvas_p0[1], sx0, canvas_p1[1], axis_col, 2.0)
        draw_list.add_line(canvas_p0[0], sy0, canvas_p1[0], sy0, axis_col, 2.0)

        # --- 绘制期望轨迹 (橙色) ---
        color_orange = imgui.get_color_u32_rgba(1.0, 0.55, 0.0, 0.8)
        if len(self.expected_polyline) >= 2:
            pts = [self._w2s(px, py, canvas_p0, canvas_size) for px, py in self.expected_polyline]
            draw_list.add_polyline(pts, color_orange, False, 2.0)

            # Expected trajectory start marker (t=0): should be the right-most point at default yaw=0.
            sx0_w, sy0_w = self._circle_xy_at_time(0.0, self.circle.radius, self.circle.speed, self.circle.ramp)
            c = math.cos(self.anchor_yaw)
            s = math.sin(self.anchor_yaw)
            start_x_world = self.anchor_x + c * sx0_w + s * sy0_w
            start_y_world = self.anchor_y + s * sx0_w - c * sy0_w
            asx, asy = self._w2s(start_x_world, start_y_world, canvas_p0, canvas_size)
            draw_list.add_circle_filled(asx, asy, 4.5, color_orange)

        # --- 绘制实际轨迹与状态 ---
        with self.lock:
            points = list(self.trail)
            pos = self.latest_pos
            yaw = self.latest_yaw

        if len(points) >= 2:
            for i in range(1, len(points)):
                t1, x1, y1 = points[i]
                age = now - t1

                if self.show_full_trail: alpha = 1.0
                elif age <= self.fade_start_s: alpha = 1.0
                elif age >= self.fade_end_s: alpha = 0.0
                else: alpha = 1.0 - (age - self.fade_start_s) / (self.fade_end_s - self.fade_start_s)

                if alpha <= 0.05: continue

                sx0, sy0 = self._w2s(points[i-1][1], points[i-1][2], canvas_p0, canvas_size)
                sx1, sy1 = self._w2s(x1, y1, canvas_p0, canvas_size)

                c = imgui.get_color_u32_rgba(0.0, 0.47, 0.8, alpha)
                thick = 2.0 if age <= self.fade_start_s else 1.0
                draw_list.add_line(sx0, sy0, sx1, sy1, c, thick)

        if pos is not None:
            sx, sy = self._w2s(pos[0], pos[1], canvas_p0, canvas_size)
            color_red = imgui.get_color_u32_rgba(0.8, 0.0, 0.0, 1.0)
            draw_list.add_circle_filled(sx, sy, 5.0, color_red)

            # 目标或估计
            if self.target_source_items[self.target_source_idx] == "fitted_tangent":
                tangent_yaw = self._estimate_tangent_yaw_from_history(points, now)
                if tangent_yaw is not None:
                    exp_color = imgui.get_color_u32_rgba(0.15, 0.47, 1.0, 0.5)
                    ex_w = pos[0] + 0.8 * math.cos(tangent_yaw)
                    ey_w = pos[1] + 0.8 * math.sin(tangent_yaw)
                    sx2, sy2 = self._w2s(ex_w, ey_w, canvas_p0, canvas_size)
                    self._draw_arrow(draw_list, (sx, sy), (sx2, sy2), exp_color, 3.0)
            else:
                if target is not None:
                    # 强烈的绿色要求 -> RGBA(0, 1, 0, 1)
                    color_target = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 1.0)
                    color_target_arrow = imgui.get_color_u32_rgba(0.0, 1.0, 0.0, 0.6)

                    tsx, tsy = self._w2s(target.x, target.y, canvas_p0, canvas_size)
                    draw_list.add_circle_filled(tsx, tsy, 5.0, color_target)

                    target_arrow_len_m = max(0.25, min(2.5, 0.25 + 0.45 * target.speed))
                    tx2_w = target.x + target_arrow_len_m * math.cos(target.yaw)
                    ty2_w = target.y + target_arrow_len_m * math.sin(target.yaw)
                    tsx2, tsy2 = self._w2s(tx2_w, ty2_w, canvas_p0, canvas_size)
                    self._draw_arrow(draw_list, (tsx, tsy), (tsx2, tsy2), color_target_arrow, 3.0)

            # 车辆朝向
            if yaw is not None:
                x2_w = pos[0] + 0.6 * math.cos(yaw)
                y2_w = pos[1] + 0.6 * math.sin(yaw)
                sx2, sy2 = self._w2s(x2_w, y2_w, canvas_p0, canvas_size)
                self._draw_arrow(draw_list, (sx, sy), (sx2, sy2), color_red, 2.0)

        draw_list.pop_clip_rect()
        imgui.end_child()
        imgui.end()

    def run(self):
        if not glfw.init():
            print("Failed to initialize OpenGL context (GLFW).")
            sys.exit(1)

        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

        window = glfw.create_window(1280, 860, "PX4 Live XY Viewer (PyImGui)", None, None)
        glfw.make_context_current(window)

        if not window:
            glfw.terminate()
            print("Failed to initialize GLFW window.")
            sys.exit(1)

        # 初始化 ImGui
        imgui.create_context()
        impl = GlfwRenderer(window)

        while not glfw.window_should_close(window) and self.running:
            glfw.poll_events()
            impl.process_inputs()

            imgui.new_frame()

            # 渲染所有的 UI 和 自定义画板
            self._render_ui()

            gl.glClearColor(0.1, 0.1, 0.1, 1)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)

            imgui.render()
            impl.render(imgui.get_draw_data())
            glfw.swap_buffers(window)

        self.running = False
        impl.shutdown()
        glfw.terminate()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--link", default="udpin:0.0.0.0:14550")
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()

    app = LiveViewerImGui(link=args.link, rate_hz=args.rate)
    app.run()


if __name__ == "__main__":
    main()
