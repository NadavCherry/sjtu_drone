#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grid keyboard controller (cardinal, robust stop + PD yaw)
- Right  → North (Y+)
- Up     → East  (X+)
- Left   → South (Y-)
- Down   → West  (X-)
- z: CCW 90°  |  c: CW 90°
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import yaml, cv2, numpy as np, math, tkinter as tk, os, time

# ---------- tuning ----------
RESERVED_RATE_HZ   = 25            # control loop rate
POS_KP             = 1.2           # [1/s] for world-frame vx,vy
V_MAX              = 0.45          # [m/s] cap on linear speed
SLOW_RADIUS        = 0.35          # [m] begin slowing near goal
STOP_POS_TOL       = 0.05          # [m] position tolerance
STOP_V_TOL         = 0.03          # [m/s] linear speed tolerance
STOP_W_TOL         = 0.07          # [rad/s] angular speed tolerance
STOP_MAX_WAIT_S    = 3.0           # [s] max waiting for full stop

YAW_KP             = 2.8           # PD yaw controller
YAW_KD             = 0.5
W_MAX              = 1.6           # [rad/s] cap on angular speed
YAW_TOL            = 0.03          # [rad] final yaw tolerance

def clamp(x, lo, hi): return min(max(x, lo), hi)

def quat_to_yaw(q):
    siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
    cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class GridKeyboardController(Node):
    def __init__(self):
        super().__init__('grid_keyboard_controller')

        # --- load low-res map ---
        map_yaml = "/root/drone_workspace/sjtu_drone/maps/hospital_map_downscaled.yaml"
        with open(map_yaml, 'r') as f:
            info = yaml.safe_load(f)
        self.resolution = float(info['resolution'])  # ≈ 0.5m
        self.origin = info['origin']                 # [ox, oy, yaw]
        map_img = info['image']
        if not os.path.isabs(map_img):
            map_img = os.path.join(os.path.dirname(map_yaml), map_img)
        img = cv2.imread(map_img, cv2.IMREAD_UNCHANGED)
        if img is None: raise FileNotFoundError(map_img)
        self.map = np.flipud((img < 50).astype(np.uint8))  # 1=occ,0=free
        H, W = self.map.shape
        self.get_logger().info(f"Map: {W}x{H} cells, res={self.resolution}m; "
                               f"X=[{self.origin[0]:.2f},{self.origin[0]+W*self.resolution:.2f}] "
                               f"Y=[{self.origin[1]:.2f},{self.origin[1]+H*self.resolution:.2f}]")

        # --- state/IO ---
        self.pose = None
        self.yaw = 0.0
        self.vel_lin = 0.0
        self.vel_ang = 0.0

        self.sub_pose = self.create_subscription(Pose, '/simple_drone/gt_pose', self.pose_cb, 10)
        self.sub_vel  = self.create_subscription(Twist, '/simple_drone/gt_vel', self.vel_cb, 10)
        self.pub_cmd  = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)

        # --- UI ---
        self.root = tk.Tk()
        self.root.title("Drone Grid Controller (Robust)")
        self.root.geometry("420x220")
        self.msg = tk.StringVar(value="Ready")
        tk.Label(self.root, text="→ North | ↑ East | ← South | ↓ West | z CCW | c CW | space STOP", font=("Arial", 11)).pack(pady=10)
        tk.Label(self.root, textvariable=self.msg, font=("Consolas", 10)).pack()
        self.root.bind("<Key>", self.on_key)
        self.root.after(40, self.ui_spin)
        self.root.mainloop()

    # --------- callbacks ---------
    def pose_cb(self, m: Pose):
        self.pose = m.position
        self.yaw  = quat_to_yaw(m.orientation)

    def vel_cb(self, m: Twist):
        self.vel_lin = math.hypot(m.linear.x, m.linear.y)
        self.vel_ang = abs(m.angular.z)

    # --------- helpers ---------
    def set_status(self, s): self.msg.set(s); self.get_logger().info(s)
    def publish_stop(self, times=1):
        z = Twist()
        for _ in range(times):
            self.pub_cmd.publish(z)

    def world_to_map(self, x, y):
        ox, oy, _ = self.origin
        return int((x-ox)/self.resolution), int((y-oy)/self.resolution)

    def cell_center_world(self, mx, my):
        ox, oy, _ = self.origin
        return (mx+0.5)*self.resolution+ox, (my+0.5)*self.resolution+oy

    def in_bounds(self, mx, my):
        H, W = self.map.shape
        return 0 <= mx < W and 0 <= my < H

    def is_free(self, mx, my):
        return self.in_bounds(mx,my) and (self.map[my, mx] == 0)

    # --------- robust stop verifier ---------
    def wait_full_stop(self, extra_zero_cmds=True):
        t_end = time.time() + STOP_MAX_WAIT_S
        if extra_zero_cmds:
            # flood a few zeros quickly to “stick” the stop
            for _ in range(6):
                self.publish_stop()
                time.sleep(0.03)
        while rclpy.ok() and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.vel_lin < STOP_V_TOL and self.vel_ang < STOP_W_TOL:
                return True
            # reinforce stop
            self.publish_stop()
        return False

    # --------- yaw PD rotate ---------
    def rotate_to_yaw(self, target_yaw):
        rate_dt = 1.0/RESERVED_RATE_HZ
        prev_err = None
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            err = (target_yaw - self.yaw + math.pi) % (2*math.pi) - math.pi
            if abs(err) < YAW_TOL and self.vel_ang < STOP_W_TOL:
                break
            d_err = 0.0 if prev_err is None else (err - prev_err)/rate_dt
            prev_err = err
            w_cmd = clamp(YAW_KP*err + YAW_KD*d_err, -W_MAX, W_MAX)
            tw = Twist()
            tw.angular.z = w_cmd
            self.pub_cmd.publish(tw)
            time.sleep(rate_dt)
        self.publish_stop(times=3)
        self.wait_full_stop(extra_zero_cmds=False)
        self.set_status(f"Yaw aligned: {math.degrees(self.yaw):.1f}°  ({time.time()-start:.2f}s)")

    # --------- world-frame position control (vx, vy) ---------
    def drive_to_xy(self, tx, ty):
        rate_dt = 1.0/RESERVED_RATE_HZ
        start = time.time()
        while rclpy.ok() and self.pose is not None:
            rclpy.spin_once(self, timeout_sec=0.0)
            dx, dy = tx - self.pose.x, ty - self.pose.y
            dist = math.hypot(dx, dy)
            if dist < STOP_POS_TOL:  # reached
                break

            # position->velocity (world frame)
            k = POS_KP
            if dist < SLOW_RADIUS:
                k *= (dist / SLOW_RADIUS)  # slow down near target

            vx = clamp(k * dx, -V_MAX, V_MAX)
            vy = clamp(k * dy, -V_MAX, V_MAX)

            tw = Twist()
            tw.linear.x = vx
            tw.linear.y = vy
            self.pub_cmd.publish(tw)
            time.sleep(rate_dt)

        # stop + verify
        self.publish_stop(times=6)
        ok = self.wait_full_stop(extra_zero_cmds=False)
        self.set_status(f"Arrived (ok={ok})  "
                        f"pos_err≈{math.hypot(tx-(self.pose.x if self.pose else tx), ty-(self.pose.y if self.pose else ty)):.3f}  "
                        f"v={self.vel_lin:.3f} ω={self.vel_ang:.3f}  "
                        f"time={time.time()-start:.2f}s")

    # --------- cardinal helpers ---------
    def yaw_for_cardinal(self, card):
        return {'E':0.0,'N':math.pi/2,'W':math.pi,'S':-math.pi/2}[card]

    def delta_for_cardinal(self, card):
        return {'E':(1,0),'N':(0,1),'W':(-1,0),'S':(0,-1)}[card]

    def step_cardinal(self, card):
        if self.pose is None:
            self.set_status("Waiting for pose…"); return
        mx, my = self.world_to_map(self.pose.x, self.pose.y)
        dx, dy = self.delta_for_cardinal(card)
        tmx, tmy = mx+dx, my+dy
        if not self.in_bounds(tmx,tmy): self.set_status("OOB"); return
        if not self.is_free(tmx,tmy):   self.set_status("Blocked"); return

        # 1) align yaw robustly
        tyaw = self.yaw_for_cardinal(card)
        self.set_status(f"Rotate → {card}")
        self.rotate_to_yaw(tyaw)

        # 2) go to exact cell center with world-frame controller
        tx, ty = self.cell_center_world(tmx, tmy)
        self.set_status(f"Move → {card} center ({tx:.2f},{ty:.2f})")
        self.drive_to_xy(tx, ty)

    # --------- key bindings ---------
    def on_key(self, e):
        k = e.keysym
        if   k == 'Right': self.step_cardinal('N')  #
        elif k == 'Up':    self.step_cardinal('E')  #
        elif k == 'Left':  self.step_cardinal('S')  #
        elif k == 'Down':  self.step_cardinal('W')  #
        elif k == 'z':     # CCW 90°
            self.rotate_to_yaw(((self.yaw + math.pi/2) + math.pi)%(2*math.pi)-math.pi)
        elif k == 'c':     # CW  90°
            self.rotate_to_yaw(((self.yaw - math.pi/2) + math.pi)%(2*math.pi)-math.pi)
        elif k == 'space':
            self.publish_stop(times=8)
            self.wait_full_stop()
            self.set_status("STOP")

    # --------- UI loop ---------
    def ui_spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        self.root.after(40, self.ui_spin)

def main():
    rclpy.init()
    GridKeyboardController()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
